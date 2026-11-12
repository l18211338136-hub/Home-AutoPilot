# HomeFly Logging 基础设施规范

日志与通信、传感器、执行器一样，属于**所有模块都可以使用的公共基础设施**。从第一阶段 ESP32-S3 就建立统一接口，后续迁移到 STM32、Linux 或 PX4 风格架构时，业务模块不需要修改一行代码。

## 1. 分两部分：平台无关前端 + 平台相关后端

```
src/platforms/
├── common/logging/          平台无关：统一 API，所有飞控模块调用
│   ├── log_level.hpp        五个日志级别
│   ├── log_sink.hpp         ILogSink 输出通道抽象
│   ├── log_backend.hpp      平台默认 Console sink 工厂声明
│   ├── log.hpp / log.cpp    统一 API、宏、过滤、限流、环形缓冲、统计
│   └── flight_data.hpp/.cpp 独立的结构化飞行数据二进制记录
│
├── esp32/logging/log_backend.cpp    USB CDC / UART0 输出
├── stm32/logging/log_backend.cpp    UART / ITM 输出
└── linux/logging/log_backend.cpp    stdout 输出
```

`common` 里**没有任何平台 API**；`esp32` / `stm32` 目录里只有输出实现。每个固件只链接一个 backend，链接期绑定 `log_backend_console()`，无运行时分发开销。

> 目录放在 `src/platforms/` 下，是为了与 SKILL.md 既有的 `src/` 分层约定保持一致；职责上 common 是平台无关的公共前端。

## 2. 统一日志级别

| 级别 | 用途 | 编译期剔除阈值 |
| --- | --- | --- |
| `Debug` | 调试信息 | `HF_LOG_COMPILE_MIN_LEVEL=1` |
| `Info` | 正常运行信息 | `HF_LOG_COMPILE_MIN_LEVEL=2` |
| `Warn` | 警告信息 | `HF_LOG_COMPILE_MIN_LEVEL=3` |
| `Error` | 错误信息 | `HF_LOG_COMPILE_MIN_LEVEL=4` |
| `Fatal` | 致命错误 | **永不过滤** |

`Fatal` 不做编译期剔除：致命错误日志在任何构建配置下都必须保留。

## 3. 统一 API

```cpp
#include "platforms/common/logging/log.hpp"
#include "platforms/common/logging/log_backend.hpp"

homefly::LogConfig config{};
config.min_level = homefly::LogLevel::Info;
config.rate_limit_ms = 500;
homefly::log_init(clock, homefly::log_backend_console(), config);

HF_LOG_INFO("SENSOR", "MPU6050 initialized");
HF_LOG_WARN("BATTERY", "Low voltage: %.2fV", voltage);
HF_LOG_ERROR("MOTOR", "Motor initialization failed");
HF_LOG_FATAL("SYSTEM", "unrecoverable fault");

// 低优先级 / 空闲任务：
homefly::log_drain(8);
```

设计要点：

- 业务模块**只允许**使用 `HF_LOG_*` 宏，禁止 `printf` / `cout` / `ESP_LOGx` / `HAL_UART_Transmit`。
- `log_init()` 采用依赖注入（`IClock&` + `ILogSink&`），因此 Linux Native 可以注入 `SimClock` 和假 sink，日志输出完全可确定、可测试。
- 编译期级别过滤通过 `HF_LOG_COMPILE_MIN_LEVEL` 把低等级调用**整体消除**（宏展开为空），生产固件零开销。注意：被剔除时格式化参数不求值，因此不要在日志参数里放副作用。
- 运行期级别过滤用 `log_set_level()` 动态调整，不重编译。

## 4. 输出格式

```
[00001234] [INFO ] [SYSTEM] HomeFly booting
[00001240] [INFO ] [SENSOR] MPU6050 initialized
[00001245] [INFO ] [MOTOR] Motor driver initialized
[00001300] [WARN ] [BATTERY] Low voltage: 3.45V
[00001320] [ERROR] [IMU] Sensor read failed
```

| 字段 | 说明 |
| --- | --- |
| `[00001234]` | 系统启动后的**毫秒数**，8 位零填充，来自注入的 `IClock::now_us()/1000` |
| `[INFO ]` | 5 字符定宽级别标签 |
| `[SYSTEM]` | TAG（模块名，最长 12 字符） |
| 余下 | MESSAGE，`vsnprintf` 有界格式化，整行硬上限 160 字节 |

Console sink 负责追加换行；`ILogSink::write()` 是二进制安全的，文本与二进制通道共用同一抽象。

## 5. 输出通道架构

```
飞控业务模块  IMU / MOTOR / BATTERY / FLIGHT / NAVIGATION
        │  HF_LOG_*(tag, fmt, ...)
        ▼
Common Logging API   级别过滤 → 限流 → 有界格式化 → 无锁 MPMC 环形缓冲
        │  ← 控制循环到此为止，绝不碰串口
        ▼  log_drain() 由低优先级任务驱动
ILogSink 通道   Console(USB CDC/UART) │ File(SD/Flash) │ Telemetry(无线)
        ▼
platforms/<platform>/logging/log_backend.cpp
```

第一阶段 ESP32-S3 只实现 USB CDC / UART 输出。后续增加通道的业务代码改动量为 **0**：

```cpp
SdCardLogSink sd_sink;      // 飞行日志，二进制 + 文本
TelemetryLogSink radio_sink; // 关键事件下发地面站
homefly::log_add_sink(sd_sink);
homefly::log_add_sink(radio_sink);
```

`log_init()` 自动注册平台 Console sink，最多 4 个 sink（编译期常量，无动态分配）。

## 6. 实时性：日志绝不能阻塞控制循环

姿态控制循环运行在 500 Hz–1 kHz。若每次串口同步输出，控制周期必然被拖垮。因此**第一阶段就落地**以下机制（不是留到后续）：

| 要求 | 实现 |
| --- | --- |
| 环形缓冲区 | 静态 32 槽 × 160 字节的 **Vyukov 无锁 MPMC 队列**，支持多任务并发写、后台任务并发读 |
| 异步输出任务 | 前端 `log_write()` 只入队；`log_drain()` 由低优先级/空闲任务调用，串口/文件/无线 IO 只在这里发生 |
| 日志等级过滤 | 编译期 `HF_LOG_COMPILE_MIN_LEVEL` + 运行期 `LogConfig::min_level` 双层 |
| 高频日志限流 | `rate_limit_ms`：按 `(tag, level)` 哈希到 16 个槽，间隔内重复日志直接丢弃并计数 |
| 串口发送失败统计 | `LogStats::backend_failures`、`bytes_written`，可由遥测上报 |
| 实时数据与调试日志分离 | `FlightDataRecorder` 独立二进制 SPSC 环 + `FlightLogWriter` 批量落盘（见下节与 [flight_log.md](flight_log.md)） |

`log_write()` 的硬保证：

- 无动态内存分配（全部静态存储）；
- 无阻塞、无锁等待、无 I/O；
- 环形缓冲满时**丢弃 + 计数**，绝不等待；
- 格式化有硬上界（`vsnprintf` 到固定 160 字节槽）；
- 全部成员函数 `noexcept`。

> 权衡说明：`vsnprintf` 是纯 CPU 运算且有界，放在前端；真正耗时且不确定的串口/文件写入放在后端。浮点格式化较多时仍应优先使用结构化二进制通道。

**Linux Native 的落地方式**：仿真用一条真实的 `std::thread` 低优先级任务同时驱动 `log_drain(8u)` 与 `FlightLogWriter::pump()`，这是**唯一发生串口/文件 I/O 的地方**。控制循环里 `HF_LOG_*` 只入队、飞行数据只 `push()`，每周期仅做一次 `std::this_thread::yield()` 调度让步（不是 I/O），并在飞行数据环占用过半时做有界背压等待（仅 host 全速仿真需要，真机 1 kHz 周期自带空闲时间）。让后台任务及时排空两个环。启动阶段和收尾阶段各做一次单线程 `log_drain()`，`log_shutdown()` 内部还有兜底 drain。

**真机上的异步输出任务**（ESP32-S3 FreeRTOS 示例）：

```cpp
static void logging_task(void*)
{
    for (;;)
    {
        homefly::log_drain(8u);                       // 文本日志：串口输出
        homefly::flight_log.pump(recorder, 16u);      // 飞行日志：批量写 SD
        vTaskDelay(pdMS_TO_TICKS(10));                // 低优先级，永不抢占控制任务
    }
}
// app_main(): xTaskCreatePinnedToCore(logging_task, "log", 4096, nullptr, 1, ...);
// 退出/断臂时: flight_log.finish(recorder.dropped()); flight_log_storage_close();
```

控制任务的优先级必须高于日志任务；`log_write()` 的 MPMC 队列允许多任务并发写，无需加锁。STM32 裸机可把这两句放进主循环空闲分支，其余逻辑完全不变。

统计快照：

```cpp
const homefly::LogStats stats = homefly::log_get_stats();
// emitted / drained / dropped_level / dropped_rate / dropped_full
// backend_failures / bytes_written
```

## 7. 结构化飞行数据与飞行日志（与普通日志严格分离）

IMU 原始值、姿态角、电机输出等高频数据**禁止**使用 `HF_LOG_*` 文本日志，否则字符串格式化会吃掉整个 CPU 预算并瞬间填满日志环。

```cpp
homefly::FlightDataRecord record{};   // POD：magic/version/mode/timestamp/attitude/gyro/accel/motor
recorder.push(record);                // 控制循环：一次 POD 拷贝，零 I/O
flight_log.pump(recorder, 16u);       // 低优先级任务：攒 16 条一次落盘
flight_log.finish(recorder.dropped()); // 退出：刷尾块 + 写 Footer + flush
```

- 独立 128 槽 SPSC 环形缓冲，单生产者（控制循环）/ 单消费者（后台任务）。
- 固定二进制布局，带 `magic` / `version`；`mode` 用 `uint8_t`，避免 `platforms/common` 反向依赖 `FlightCore`。
- 满时丢弃并计数；sink 写入失败时保留记录供下次重试。
- `FlightLogWriter` 把记录批量写入自描述的 `.hfl` 文件（64 B Header / 16 条一块 / 32 B Footer + FNV-1a 校验和），格式与平台后端规范见 [flight_log.md](flight_log.md)。

Linux Native **真实落盘**：仿真 2000 周期生成 `144096` 字节 `.hfl`（2000 条记录、125 块、`dropped=0`），同期文本日志只有 15 条；`tools/hfl_dump.py` 可校验 checksum、检测截断/崩溃会话并导出 CSV。

## 8. 平台后端实现指南

| 平台 | 文本日志后端 | 飞行日志存储 |
| --- | --- | --- |
| ESP32-S3 | `esp32/logging/log_backend.cpp`：UART0，`uart_write_bytes` | `esp32/logging/flight_log_sink.cpp`：SD/FatFS BSP 钩子（占位） |
| STM32 | `stm32/logging/log_backend.cpp`：`HAL_UART_Transmit` | `stm32/logging/flight_log_sink.cpp`：SD/QSPI BSP 钩子（占位） |
| Linux Native | `linux/logging/log_backend.cpp`：stdout | `linux/logging/file_log_sink.cpp`：真实 `.hfl` 文件 ✅ |

平台存储统一通过 `flight_log_storage_open/…/close()` 链接期绑定，common 层不含任何 SDK 头文件。MCU 侧后续可换原生 USB CDC / ITM / DMA 双缓冲，业务代码不改。

ESP32 文件加入 ESP-IDF 组件 `SRCS`（`ESP_PLATFORM` 由 IDF 定义）；STM32 加入交叉编译 target 并定义 `HOMEFLY_PLATFORM_STM32`。在这两个宏未定义时，对应翻译单元为空，不会与其它平台 backend 产生符号冲突。

后端 `write()` 允许阻塞，因为它只被后台 drain 任务调用；但**不得**在中断中调用。

## 9. 测试

`test/platforms/test_logging.cpp`、`test/platforms/test_flight_data.cpp`、`test/platforms/test_flight_log.cpp` 覆盖：

- 级别过滤、输出格式（含浮点）、运行期级别调整；
- 高频限流统计；
- 环形缓冲满时丢弃且**不产生任何同步 I/O**；
- sink 失败统计、drain 条数限制、多 sink、sink 容量上限；
- 4 生产者 + 1 后台消费者的并发守恒；
- 二进制飞行记录的顺序、溢出、失败重试、重置；
- `log_shutdown()` 兜底刷出未消费的尾部记录，之后写入变为 no-op；
- 飞行日志 Header 自描述字段、16 条批量块、尾块补齐、Footer 计数与丢帧、FNV-1a 校验和独立复算、写入失败保留记录重试、未激活写入器拒绝写入。

端到端：Linux Native 实际生成 `.hfl` 后由 `tools/hfl_dump.py` 读回校验（checksum / 记录数 / 截断 / CSV 导出），可直接作为 CI 步骤。

## 10. 禁止事项

- 在控制循环里同步输出日志、调用 `log_drain()` 或 `FlightLogWriter::pump()` / `finish()`。
- 在业务模块里直接使用平台日志 API（`ESP_LOGI`、`printf`、`HAL_UART_Transmit`）。
- 用文本日志打印每周期的 IMU / 姿态 / 电机数据。
- 逐帧调用 `fwrite` / `f_write`（必须攒满 16 条一块），或让飞行日志与文本日志共用同一个环形缓冲。
- 在日志参数里放有副作用的表达式（编译期剔除时不会求值）。
- 在 `HF_LOG_*` 消息里放 unbounded 字符串（整行硬截断 160 字节）。
- 为日志引入动态内存分配或第三方依赖。
