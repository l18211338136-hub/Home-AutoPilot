# 架构说明

## 分层与依赖方向

```
Applications ──> FlightCore ──> HAL <── Drivers
      │              │            ▲       │
      │              └──> Control ┘       │
      ├──> Logging ────────────────┐      │
      └──> Configuration           │      │
                                   │      │
Platforms ──实现──> HAL 接口 ───────┴──────┘
     │
     └──> <platform>/logging/  （唯一含平台输出代码的位置）
              ├── log_backend.cpp       Console 文本输出
              └── flight_log_sink.cpp  飞行日志存储（Linux 文件 / MCU SD·Flash）
```

依赖只允许从上层指向下层的**抽象**：

| 层 | 目录 | 允许依赖 | 禁止依赖 |
| --- | --- | --- | --- |
| 应用组装 | `src/applications/` | 全部层 | 无（唯一允许实例化具体类的层） |
| 飞控核心 | `src/flight_core/` | HAL、control、configuration | ESP-IDF、STM32 HAL、Arduino、任何传感器驱动 |
| 控制算法 | `src/control/` | HAL 数据类型、标准数学库 | GPIO / SPI / I2C / PWM |
| HAL | `src/hal/` | 无（纯接口 + POD） | 任何实现、任何 SDK 类型 |
| 驱动 | `src/drivers/` | HAL 接口 | FlightCore、其它具体驱动 |
| 平台 | `src/platforms/` | HAL 接口 | FlightCore |
| 日志前端 | `src/platforms/common/logging/` | `IClock`、`ILogSink` 抽象 | 平台 SDK、FlightCore、具体串口/文件系统 |
| 日志后端 | `src/platforms/<平台>/logging/` | 日志前端抽象（含 `flight_log_storage_*` 工厂） | FlightCore、业务模块 |
| 通信 | `src/communication/` | `ILink`、核心数据类型 | 具体 UART / 无线芯片 |
| 配置 | `src/configuration/` | control 的增益结构 | 硬件 |
| 仿真 | `simulation/` | HAL、control | FlightCore（由应用组装）、Gazebo SDK |
| 仿真源 | `src/applications/linux_native/simulation_source.*` | 全部层 | 无（唯一选择内置 plant / 外部 SITL 的位置） |

## 依赖注入

`FlightController` 只持有引用，不 `new`、不 `make_unique`、不访问单例：

```cpp
FlightController(
	IImu& imu,
	IMotor& motor,
	IClock& clock,
	const FlightConfig& config
);
```

`src/applications/linux_native/main.cpp` 是唯一的组装根（Composition Root）：

```cpp
SimPlant plant;
SimImu imu(plant, clock);
SimMotor motor(plant);
FlightController controller(imu, motor, clock, config);
```

换成真实硬件时，只需替换 `SimImu` / `SimMotor` / `IClock` 的实现，`FlightController` 一行不改。高频路径如需零开销抽象，可在保持接口契约的前提下改用模板静态多态，但不为统一形式牺牲实时性。

## 一个控制周期的数据流

```
IClock::now_us
   ↓
SafetyMonitor::check_timing ─── 超时 ──→ Failsafe
   ↓
IImu::read                 ─── 读取失败 ─→ Failsafe
   ↓
SafetyMonitor::validate_imu ─ 非有限/过期 ─→ Failsafe
   ↓
AttitudeEstimator::update   （首帧用重力向量对齐）
   ↓
FlightStateMachine::update  （Arm / Disarm / 故障事件）
   ↓  Stabilize
P(angle) → 期望角速度 → PID(rate) → 三轴力矩指令
   ↓
MotorMixer::mix            （Quad-X，输出限幅到 [0,1]）
   ↓
IMotor::write              ─── 失败 ────→ Failsafe
```

任何异常分支都收敛到同一个 `enter_failsafe()`：断臂 + 输出最小值 + 状态置粘性 `Failsafe`。恢复必须由显式的 `disarm_request` 且传感器有效才能完成。

## 关键类型

| 类型 | 位置 | 说明 |
| --- | --- | --- |
| `ImuData` | `hal/i_imu.hpp` | 硬件数据单元，gyro rad/s，accel m/s²，z 轴向上 |
| `Attitude` | `control/attitude_estimator.hpp` | roll>0 右倾、pitch>0 抬头、yaw>0 右转 |
| `VehicleState` / `Setpoint` / `ControlOutput` | `flight_core/types.hpp` | 核心数据结构 |
| `FlightConfig` | `configuration/flight_config.hpp` | 所有可调参数 |
| `FailsafeReason` | `flight_core/types.hpp` | 故障原因枚举，便于遥测上报 |
| `LogLevel` / `LogConfig` / `LogStats` | `platforms/common/logging/log.hpp` | 统一日志级别、配置与丢弃/失败统计 |
| `ILogSink` | `platforms/common/logging/log_sink.hpp` | 二进制安全输出通道抽象 |
| `FlightDataRecord` | `platforms/common/logging/flight_data.hpp` | 独立的结构化飞行数据 POD |
| `FlightLogHeader` / `FlightLogFooter` / `FlightLogWriter` | `platforms/common/logging/flight_log.hpp` | 自描述 `.hfl` 格式、批量块写入、校验和与统计 |
| `flight_log_storage_*` | `platforms/common/logging/flight_log_storage.hpp` | 平台存储工厂（链接期绑定，每平台一份实现） |

## 日志基础设施

日志是**所有层共用的横切基础设施**，因此不挂在 FlightCore 下，而放在 `src/platforms/common/logging/`：平台无关的前端 + 各平台一个 `log_backend.cpp`。业务模块只依赖 `HF_LOG_*` 宏与 `ILogSink` 抽象，迁移平台时零改动。

控制循环中日志调用只做无锁入队；`log_drain()` 与 `FlightLogWriter::pump()` 由同一个低优先级任务驱动所有实际 I/O。高频飞行数据走 `FlightDataRecorder` 二进制环 + `FlightLogWriter` 16 条批量落盘，与文本日志物理隔离，生成可离线校验的 `.hfl` 文件。详见 [logging.md](logging.md) 与 [flight_log.md](flight_log.md)。

## 扩展点

- **新增传感器**：在 `src/hal/` 定义能力接口，在 `src/drivers/<类别>/drv_*.cpp` 实现，应用组装层替换实例。
- **新增 MCU 平台**：在 `src/platforms/<平台>/` 实现 `IClock` / `IUart` / `IGpio` 等，提供 `logging/log_backend.cpp`（Console）与 `logging/flight_log_sink.cpp`（四个 `flight_log_storage_*` 工厂函数 + BSP 钩子），复用全部核心库。
- **新增日志输出通道**：实现 `ILogSink`（SD 卡、Flash、无线遥测），启动时 `log_add_sink()` 注册，业务模块不改；飞行日志换存储介质只需替换平台工厂实现，`.hfl` 格式与离线工具不变。
- **新增通信协议**：实现 `ILink`，替换 `TelemetryReporter` 的编码，不动 `IUart`。
- **新增控制算法**：在 `src/control/` 增加纯数据算法类，由 `FlightController` 组合。
- **新增外部仿真器**：在 `simulation/<仿真器>/` 实现 `IImu` / `IMotor` / `IClock` 适配器，并在 `SimulationSource` 增加一个实现；主循环、日志、遥测、飞行日志零改动。Gazebo 已按此方式接入，见 [gazebo.md](gazebo.md)。

## 外部仿真器（SITL）

`SimulationSource` 抽象出「一个控制周期如何开始/结束」，把内置确定性 plant 与外部仿真统一到同一条主循环：

| 实现 | 时间来源 | 物理推进 | I/O 位置 |
| --- | --- | --- | --- |
| `LocalSimSource` | `SimClock` 虚拟时间（或宿主时钟） | `SimPlant::step()` | 无 |
| `GazeboSource` | `GazeboClock`（Gazebo 仿真时间戳） | Gazebo 自身 | `GazeboTransport` 低优先级任务 |

`GazeboImu` / `GazeboMotor` 只读写 SPSC 环，UDP 收发全部在 `GazeboTransport` 后台任务；`FlightController` 无法感知自己运行在仿真器中。`GazeboClock::now_us()` 由**被消费的 IMU 帧时间戳**推进，保证 `SafetyMonitor` 的陈旧样本校验与仿真时间同源。线协议、端口与运行方式见 [gazebo.md](gazebo.md)。
