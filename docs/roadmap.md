# 研发路线

遵循「先飞起来，再逐步完善自研飞控系统」。每个阶段必须有可验证的交付结果，不允许在基础飞行能力尚未验证时过早扩展复杂功能。

## Phase 1 — 架构验证（已完成 ✅）

**交付**

- 完整分层骨架：HAL / Control / FlightCore / Configuration / Communication / Drivers / Platforms / Applications。
- **统一日志基础设施**：五个级别、`HF_LOG_*` 宏、毫秒时间戳、无锁 MPMC 环形缓冲、异步 drain、双层级别过滤、高频限流、丢弃/失败统计、多 sink 扩展点；ESP32-S3 / STM32 / Linux 三个后端位。
- **结构化飞行数据 + 可落盘飞行日志（Blackbox）**：独立二进制 SPSC 环、16 条批量块缓冲、自描述 `.hfl`（Header/Footer/FNV-1a 校验和）、`tools/hfl_dump.py` 离线校验与 CSV 导出；控制循环零 I/O。
- Linux Native 独立构建（C11 + C++20，CMake + Ninja）。
- 确定性闭环仿真：SimClock → SimImu → 姿态解算 → 级联 PID → Quad-X 混控 → SimMotor → SimPlant。
- **Gazebo SITL 传输**：`SimulationSource` 统一内置 plant / 外部仿真；64 字节自描述 UDP 帧、`GazeboTransport` 后台任务 + SPSC 环、`GazeboImu/IMotor/Clock` HAL 适配器；`tools/gazebo_bridge.py` 的 `mock` 后端无需 Gazebo 即可端到端跑通锁步闭环。
- 全量构建通过，13 / 13 CTest（91 用例）通过；2000 周期实际生成并读回校验 144096 字节飞行日志。
- 稳定场景、IMU 掉线失效保护场景、Gazebo SITL 正常与失效保护四个场景均 `PASS`。

**验证命令**

```bash
cmake -S . -B build/linux -G Ninja -DHOMEFLY_PLATFORM=linux
cmake --build build/linux
ctest --test-dir build/linux --output-on-failure
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000
./build/linux/src/applications/linux_native/homefly_linux_native --steps 1000 --failsafe
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000 --verbose --throttle 500
python tools/hfl_dump.py homefly_flight.hfl --csv flight.csv --tail 10

# Gazebo SITL（两个终端，mock 后端无需安装 Gazebo）
python tools/gazebo_bridge.py --backend mock --steps 2000
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000 --gazebo
```

## Phase 2 — ESP32-S3 平台适配 + 真实 Gazebo 联调（未开始）

- 在 Linux + Gazebo Garden 上实测 `tools/gazebo_bridge.py --backend gz`：校准 IMU 话题、`Actuators.velocity` 单位、轴系映射与 1 kHz 时间契约；`--backend classic` 按 Gazebo Classic 版本补齐订阅/发布接线。
- 把 SITL 从单机 `127.0.0.1` 扩展为跨机传输时，评估 UDP 丢包/乱序（协议已有 `seq` 与校验和，传输层尚无可靠性保证）。
- `src/platforms/esp32/esp32_clock.cpp`：基于 `esp_timer_get_time()` 实现 `IClock`。
- 把 `src/platforms/esp32/logging/log_backend.cpp` 与 `flight_log_sink.cpp` 加入 ESP-IDF 组件 `SRCS`，接入 UART0 / USB CDC 输出；BSP 提供四个 `hf_flight_log_*` 钩子（`esp_vfs_fat_sdmmc_mount` + FatFS）；用真实 RTOS 低优先级任务同时调用 `log_drain(8)` 与 `flight_log.pump(recorder, 16)`。
- `src/drivers/imu/drv_mpu6050.*`、`drv_icm42688.*`：实现 `IImu`。
- `src/drivers/motor/drv_dshot.*` 或 `drv_pwm.*`：实现 `IMotor`。
- ESP-IDF 组件引用公共静态库源码，`idf.py` 构建固件。
- **验证**：台架（卸桨）读取 IMU 原始数据、姿态角与电机输出，记录日志并与仿真对照。

## Phase 3 — 首次稳定飞行（未开始）

- 实测 `step()` WCET 与抖动，确认 1 kHz 周期余量。
- 参数整定、解锁/锁定流程、遥控器链路与失控保护。
- **验证**：系留/低高度悬停录像 + 黑匣子日志。

## Phase 4 — 多平台与自研硬件（未开始）

- STM32F411 / F405 / F7 / H7 通过 `src/platforms/stm32/` 适配，含 `stm32/logging/log_backend.cpp` 与 `flight_log_sink.cpp`（FatFs / QSPI BSP 钩子，建议预分配连续文件 + DMA 双缓冲）。
- 新增无线遥测 sink（只实现 `ILogSink` + `log_add_sink()`，业务模块零改动）；飞行日志落盘与回放工具已在 Phase 1 完成，此处只做 MCU 侧适配与掉电完整性实测。
- 自研 PCB 复用现有 HAL / Drivers / FlightCore。
- **验证**：同一份 FlightCore 源码在两个 MCU 上通过相同测试集。

## Phase 5 — 能力扩展（未开始）

飞行模式、自动返航、室内定位、视觉辅助、机库对接、自动换电、多传感器融合（四元数 EKF 替代当前互补滤波）。

## 明确不做的事

- 不为统一构建而破坏 ESP-IDF / STM32Cube 的官方工作方式。
- 不在 FlightCore 里放任何平台代码。
- 不为快速实现功能在核心层引入具体传感器或第三方依赖。
- 不在基础飞行能力验证通过前扩展复杂功能。
