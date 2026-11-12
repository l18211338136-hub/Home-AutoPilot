# HomeFly 飞行日志（Blackbox）规范

飞行日志是**独立于文本日志的二进制飞行数据记录**：把每个控制周期的 IMU、姿态、电机输出完整落盘，用于事后分析、调参与事故复盘。它与 `HF_LOG_*` 调试日志物理隔离——高频数据用文本打印会在 500 Hz–1 kHz 下吃光 CPU 并瞬间填满日志环。

> 从 Phase 1 起就是**可运行的交付**：Linux Native 会真实生成 `.hfl` 文件，不是内存占位；`tools/hfl_dump.py` 可离线校验并导出 CSV。

## 1. 文件布局

```
[ FlightLogHeader   64 B ]   自描述文件头
[ block 0        ≤16*72 B ]  批量记录
[ block 1        ≤16*72 B ]
...
[ FlightLogFooter   32 B ]   计数 / 丢帧 / 写失败 / 校验和
```

全部为原生小端 POD，无压缩、无变长字段、无动态分配。离线工具只需读 `header.version` 与 `header.record_size` 即可解析，无需固件头文件。

2000 周期的实际文件：`64 + 2000*72 + 32 = 144096` 字节。

### Header（64 B，`flight_log.hpp`）

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | `magic` | u32 | `0x48464C47` = `"HFLG"` |
| 4 | `version` | u16 | 当前 `1` |
| 6 | `header_size` | u16 | `64` |
| 8 | `session_id` | u64 | 会话标识（Linux 用 Unix 秒；MCU 可用 RTC / 递增计数器） |
| 16 | `start_time_us` | u64 | 首帧时间戳 |
| 24 | `record_size` | u32 | `sizeof(FlightDataRecord)` = `72` |
| 28 | `control_period_us` | u32 | 控制周期，用于离线换算频率 |
| 32 | `config_hash` | u32 | 本架飞机调参指纹（FNV-1a），改 PID 后日志可追溯 |
| 36 | `platform[16]` | char | `"linux_native"` / `"esp32_s3"` / `"stm32_f411"` |
| 52 | `reserved[3]` | u32 | 对齐与后续扩展 |

> `uint64_t` 成员前置，保证 32 位 MCU 与 64 位主机布局一致，`static_assert(sizeof == 64)`。

### Record（72 B，`flight_data.hpp`）

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `magic` / `version` / `mode` / `reserved` | u32/u8/u8/u16 | `"HFFD"`、版本、飞行模式（`uint8_t`，避免 common 层反向依赖 FlightCore） |
| `timestamp_us` | u64 | 注入的 `IClock` 单调时间 |
| `roll` / `pitch` / `yaw` | f32 ×3 | 弧度 |
| `gyro[3]` | f32 | rad/s |
| `accel[3]` | f32 | m/s² |
| `motor[4]` | f32 | 归一化 `[0,1]` |

尾部 4 字节为结构体对齐填充，同样参与校验和、同样写入文件；解析时按 `record_size` 步进，只解读前 68 个有效字节。

### Footer（32 B）

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `magic` | u32 | `0x48464C45` = `"HFLE"` |
| `record_count` | u32 | 实际写入记录数 |
| `end_time_us` | u64 | 末帧时间戳 |
| `dropped_records` | u32 | 前端 SPSC 环溢出丢弃数 |
| `write_failures` | u32 | 后端写入失败次数 |
| `checksum` | u32 | 对所有记录字节的 FNV-1a |
| `reserved` | u32 | 保留 |

**Footer 缺失即代表会话异常终止**（断电、崩溃、未调用 `finish()`）。离线工具仍会解析所有完整记录，但把校验结果标为 `FAIL`。`write_failures > 0` 或 `checksum` 不匹配同样判为不可信日志。

## 2. 两级缓冲与实时性

```
控制循环 (1 kHz)         低优先级日志任务              存储
     │                          │                       │
     │ push(POD)                 │                       │
     ▼                          │                       │
FlightDataRecorder              │                       │
128 槽 SPSC 无锁环  ──pump(16)──▶ FlightLogWriter        │
     │  满即丢弃+计数          16 条 / 1152 B 块缓冲 ────▶ ILogSink
     │                          │                  文件 / SD / Flash
     └── 零 I/O ────────────────┴── 唯一发生 I/O 的位置 ──┘
```

硬约束：

- 控制循环里**只有一次 POD 拷贝**（`push()`）：无格式化、无锁、无动态分配、无文件/串口 I/O。
- `FlightLogWriter` 实现 `ILogSink`，由 `FlightDataRecorder::drain_to(*writer)` 逐条喂入，攒满 16 条才调用一次 `storage.write()`；相比逐帧 `fwrite` 把写调用降低 16 倍。
- 后端写入失败时 `write()` 返回 `false`，`drain_to()` 会**保留该记录**供下次 `pump()` 重试，不静默丢数据；失败计入 `write_failures`。
- 环形缓冲满时丢弃并计数，**绝不等待**；`dropped_records` 写进 Footer。
- `finish()` 刷出尾部不足一块的记录、写 Footer、`flush()`。

Linux Native 用真实的 `std::thread` 低优先级任务承载 `pump()` 与 `log_drain()`，控制循环每周期只做一次 `std::this_thread::yield()`（调度让步，不是 I/O），并在环占用过半时做有界背压等待（仿真专属，见第 6 节）。这正好等价于真机上「控制任务执行完 step 挂起 → RTOS 调度日志任务」的模型。ESP32-S3 用 FreeRTOS 低优先级任务、STM32 用空闲钩子替换该线程，业务代码零改动。

> 说明：这里的「双缓冲」落实为**前端 SPSC 环 + 后端块缓冲**两级。同步文件 I/O 下第二个块缓冲没有并发收益；真机若用 SD 卡 DMA 异步提交，可在 BSP 钩子内部再做一个 DMA 双缓冲，common 层接口不变。

## 3. 平台后端

`FlightLogWriter` 只依赖 `ILogSink&`。平台存储通过四个链接期绑定的函数提供（`flight_log_storage.hpp`），无运行时分发：

```cpp
bool flight_log_storage_open(const char* path) noexcept;
ILogSink& flight_log_storage() noexcept;
void flight_log_storage_close() noexcept;
```

| 平台 | 实现 | 状态 |
| --- | --- | --- |
| Linux Native | `file_log_sink.cpp`：`fopen("wb")` / `fwrite` / `fflush` | ✅ 可用 |
| ESP32-S3 | `esp32/logging/flight_log_sink.cpp`，BSP 提供 `hf_flight_log_mount/write/flush/close` 四个 C 钩子（`esp_vfs_fat_sdmmc_mount` + FatFS） | 占位，Phase 2 |
| STM32 | `stm32/logging/flight_log_sink.cpp`，同样四个钩子（`f_mount` / `f_write` / `f_sync`，建议预分配连续文件 + DMA） | 占位，Phase 4 |

MCU 侧文件用宏守卫（`ESP_PLATFORM` / `HOMEFLY_PLATFORM_STM32`），host 构建时为空翻译单元，不会与 Linux 实现产生符号冲突。

**BSP 钩子全部由低优先级任务调用**，允许阻塞；禁止在中断或控制循环里调用。

## 4. 使用方式

```bash
# 默认记录到 ./homefly_flight.hfl
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000

# 指定路径 / 关闭记录
... --flight-log logs/flight_001.hfl
... --no-flight-log
```

应用启动写 Header，运行期间后台批量落盘，退出时写 Footer 并关闭文件。收尾日志会打印：

```
[00002000] [INFO ] [FLIGHTLOG] path=... records=2000 blocks=125 dropped=0 write_failures=0 checksum=0xD590B8D5 footer=ok
```

## 5. 离线解析

```bash
python tools/hfl_dump.py homefly_flight.hfl                 # 摘要 + 完整性校验
python tools/hfl_dump.py homefly_flight.hfl --csv flight.csv   # 导出 16 列 CSV
python tools/hfl_dump.py homefly_flight.hfl --tail 20         # 查看末段姿态
python tools/hfl_dump.py homefly_flight.hfl --strict          # CI：丢帧也判失败
```

校验内容：Header magic/版本/尺寸、每条记录 magic/版本、Footer 记录数、FNV-1a 校验和、`write_failures`、尾块截断。任一失败返回非零退出码，可直接接进 CI。CSV 列为角度（度）、时间（秒/微秒）、陀螺/加速度/电机原始值，可直接用 Pandas / Excel 画图。

纯标准库，无第三方依赖；Python 3.10+。

## 6. 已知边界

- **仿真专属背压（back pressure）**：host 全速仿真会把 2 秒的 2000 个周期在几毫秒内跑完，OS 可能根本不给低优先级日志任务调度机会，帧数会随机丢失。因此 `main.cpp` 在环占用超过一半时做**有界** `yield()` 等待后台任务追上（`100000` 次自旋上限，绝不无限等待）。这句只存在于 Linux Native 仿真组装层，真机没有：1 kHz 控制周期之间本来就有空闲时间给低优先级任务。实测连续 5 次 2000 周期与 20000 周期长跑均 `dropped=0`、checksum 完全可复现。
- Footer 的 `dropped_records` 仍为真实后端拥塞 / 写入失败保留；离线工具默认提示「log is readable but not gap free」，`--strict` 会判为失败，适合 CI。
- `--realtime`（真实 1 kHz 节拍）下飞行日志同样 `dropped=0`、`footer=ok`；但通用 OS 的定时抖动会触发 `ControlOverrun` 粘性 Failsafe，该模式只用于观察真实时钟下的日志行为，不用于判定控制收敛。
- 单文件会话，无分段滚动；长时间飞行需要 BSP 层做文件轮转（后续按 `session_id` 分卷）。
- 未压缩，1 kHz 下约 **72 KB/s**（1 分钟 ≈ 4.3 MB）；SD 卡需预留连续空间。
- `config_hash` 只覆盖 `FlightConfig` 的显式字段，不含机型/机架等外部信息。
- 断电时最多丢失当前块（16 条 = 16 ms）且 Footer 缺失；如需更高完整性，BSP 应周期性 `f_sync()`。
- ESP32-S3 / STM32 后端目前是移植占位，不是可烧录固件；上真机前仍需实测 SD 卡写入 WCET 与掉电行为。
