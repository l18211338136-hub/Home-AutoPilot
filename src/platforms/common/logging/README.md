# HomeFly Logging（平台无关前端）

与通信、传感器、执行器并列的**公共基础设施**：任何飞控模块都可以调用，且不依赖具体硬件。

```
飞控业务模块 (IMU / MOTOR / BATTERY / FLIGHT / NAVIGATION)
        │  HF_LOG_*(tag, fmt, ...)
        ▼
Common Logging API   级别过滤 → 限流 → 有界格式化 → 无锁 MPMC 环形缓冲
        │  （控制循环到此为止，绝不碰串口）
        ▼  log_drain() 由低优先级 / 空闲任务调用
ILogSink 通道         Console(USB CDC/UART) | File(SD/Flash) | Telemetry(无线)
        │
        ▼
src/platforms/<platform>/logging/log_backend.cpp
```

| 文件 | 作用 |
| --- | --- |
| `log_level.hpp` | 五个级别 `Debug / Info / Warn / Error / Fatal` |
| `log_sink.hpp` | `ILogSink` 二进制安全输出通道抽象 |
| `log_backend.hpp` | 平台默认 Console sink 工厂声明 |
| `log.hpp` / `log.cpp` | 统一 API、`HF_LOG_*` 宏、无锁环形缓冲、过滤/限流/统计 |
| `flight_data.hpp` / `flight_data.cpp` | **独立**的结构化飞行数据二进制记录通道（128 槽 SPSC 环） |
| `flight_log.hpp` / `flight_log.cpp` | 飞行日志写入器：自描述 `.hfl` Header/Footer、16 条批量块、FNV-1a 校验和、丢帧/写失败统计 |
| `flight_log_storage.hpp` | 平台存储工厂声明，由 `platforms/<平台>/logging/flight_log_sink.cpp` 链接期实现 |

## 用法

```cpp
#include "platforms/common/logging/log.hpp"
#include "platforms/common/logging/log_backend.hpp"

homefly::LogConfig config{};
config.min_level = homefly::LogLevel::Info;
config.rate_limit_ms = 500;
homefly::log_init(clock, homefly::log_backend_console(), config);

HF_LOG_INFO("SYSTEM", "HomeFly booting");
HF_LOG_WARN("BATTERY", "Low voltage: %.2fV", voltage);
HF_LOG_ERROR("IMU", "Sensor read failed");

// 低优先级任务（唯一发生 I/O 的位置）：
homefly::log_drain(8);
```

飞行日志是另一条独立通道，同样只由低优先级任务落盘：

```cpp
#include "platforms/common/logging/flight_log.hpp"
#include "platforms/common/logging/flight_log_storage.hpp"

homefly::flight_log_storage_open("homefly_flight.hfl");
homefly::FlightLogHeader header{};
header.control_period_us = 1000;
homefly::FlightLogWriter writer;
writer.begin(homefly::flight_log_storage(), header);

// 控制循环：只 push，一次 POD 拷贝，零 I/O
recorder.push(record);

// 低优先级任务：攒满 16 条一次写盘
writer.pump(recorder, 16u);

// 退出：刷尾块 + 写 Footer + flush
writer.finish(recorder.dropped());
homefly::flight_log_storage_close();
```

## 硬约束

- `log_write()` 只做有界 CPU 运算：级别检查、限流、`vsnprintf` 到固定 160 字节槽、无锁入队。**不阻塞、不分配、不做任何 I/O**。
- 串口/文件/无线写入只发生在 `log_drain()`，由低优先级任务驱动。
- 环形缓冲满时**丢弃并计数**，绝不等待（实时控制循环不可被日志拖慢）。
- 高频 IMU / 姿态 / 电机数据**禁止**使用 `HF_LOG_*`，必须走 `FlightDataRecorder` + `FlightLogWriter` 二进制通道。
- `FlightLogWriter::pump()` / `finish()` 与 `log_drain()` 一样**只允许在低优先级任务调用**，禁止逐帧 `fwrite`。
- 禁止在业务模块里直接使用 `printf` / `cout` / `ESP_LOGx` / `HAL_UART_Transmit`。

完整设计见 `docs/logging.md` 与 `docs/flight_log.md`；离线解析用 `tools/hfl_dump.py`。
