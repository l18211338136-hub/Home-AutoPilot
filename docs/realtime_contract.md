# 实时控制路径约束

飞控属于实时控制系统，确定性与安全性优先于功能丰富度。以下约定由 `FlightConfig` 与 `FlightController::step()` 共同落实。

## 周期预算

| 项 | 当前值 | 定义位置 |
| --- | --- | --- |
| 控制周期 | 1000 µs（1 kHz） | `FlightConfig::control_period_us` |
| IMU 采样有效期 | 5000 µs | `FlightConfig::imu_timeout_us` |
| 控制超时判定 | 周期 × 3 | `FlightConfig::overrun_factor` |
| 单次 `step()` 执行时间上限 | **未在 MCU 实测** | 移植时必须用 GPIO 打点测量，目标 < 1 ms |
| 数据更新时间 | 由 `ImuData::timestamp_us` 标注 | `SimImu` / 真实驱动必须填写单调时钟 |

> Linux Native 只验证逻辑正确性，不提供实时性保证。上 MCU 前必须实测最坏执行时间（WCET）与抖动。

## step() 的硬约束

- 无 `new` / `malloc` / STL 容器增长，所有对象在构造期就位。
- 无阻塞、无 `sleep`、无锁、无文件/网络 IO。
- 日志只允许 `HF_LOG_*` 前端入队（32 槽无锁 MPMC 环形缓冲 + 有界 `vsnprintf`）；控制循环内禁止 `printf`、串口写入和 `log_drain()`。串口/文件/无线 I/O 只发生在低优先级 drain 任务。
- 每周期的高频飞行数据用 `FlightDataRecorder` 二进制 POD 入队，禁止文本日志；`FlightLogWriter` 攒满 16 条一块、由低优先级任务批量落盘（`.hfl`），控制循环内禁止 `pump()` / `finish()` / 任何文件写入。详见 [flight_log.md](flight_log.md)。
- **外部 SITL**：`GazeboImu::read()` / `GazeboMotor::write()` 只读写 SPSC 环（一次 POD 拷贝），UDP `sendto` / `recvfrom` 全部在 `GazeboTransport` 低优先级任务；控制循环内禁止任何 socket 调用。每个控制周期恰好一帧 IMU / 一帧电机，仿真时间步长必须等于 `control_period_us`。详见 [gazebo.md](gazebo.md)。
- 无无界循环：所有循环上界为常量（`MotorMixer::kMotorCount` 等）。
- 无递归，无异常抛出路径（全部成员函数 `noexcept` 或使用返回码）。
- 不访问具体硬件 API，只调用 HAL 虚函数。
- 所有除法和 `atan2` 的输入都经过有限性与范围检查（`SafetyMonitor`）。
- `dt` 被夹在 `[1e-4 s, 0.5 s]`，避免时钟异常造成除零或输出爆炸。

## 故障与降级

| 触发条件 | 原因码 | 行为 |
| --- | --- | --- |
| `IImu::read()` 返回 false | `ImuReadFault` | 立即 Failsafe |
| 采样含 NaN/Inf、加速度模长越界 | `InvalidImuData` | 立即 Failsafe |
| 采样时间戳过期或来自未来 | `ImuTimeout` | 立即 Failsafe |
| 相邻两次 `step()` 间隔 > 3 周期 | `ControlOverrun` | 立即 Failsafe，积分器不再累积 |
| `IMotor::write()` 失败 / 通道数不符 | `MotorFault` | 立即 Failsafe |
| 姿态解算失败 | `InvalidImuData` | 立即 Failsafe |
| `Disarmed` / `Failsafe` 状态 | — | 断臂 + 全部输出 `motor_output_min` |

Failsafe 是**粘性**状态：传感器恢复后仍保持，必须由显式的 `disarm_request` 且传感器有效才能回到 `Disarmed`，再重新解锁。这避免 IMU 间歇性掉线时电机反复重启。

## 电机失效安全

- `SimMotor` 与 `PwmMotor` 都实现了硬件互锁：**未解锁时无论写入什么值，实际输出都是最小值**。
- `PwmMotor::disarm()` 主动调用 `IPwmOutput::set_all_min()`。
- `MotorMixer` 输出统一限幅到 `[motor_output_min, motor_output_max]`。
- 任何 Failsafe 分支都走 `output_safe_state()`，且会再次调用 `IMotor::disarm()`。

## 待补齐（上真机前）

- [ ] 用逻辑分析仪/GPIO 打点实测 `step()` 的 WCET 与抖动（含日志前端入队开销）。
- [ ] 实测 `log_write()` 在目标 MCU 上的最坏耗时（尤其浮点格式化），必要时把格式化也移到后台任务。
- [ ] 在 ESP32-S3 / STM32 上把 `log_drain(8)` 与 `flight_log.pump(16)` 接到真实低优先级任务或空闲钩子，并用逻辑分析仪确认控制周期内无任何串口/SD 卡活动。
- [ ] IMU 硬件 FIFO / 数据就绪中断，替代轮询 `read()`。
- [ ] 看门狗与任务优先级配置（平台层）。
- [ ] 电池电压监测与低电压降级。
- [ ] 遥控器信号丢失超时（当前 `Setpoint` 只带解锁/锁定请求）。
- [x] 黑匣子记录（禁止在高频路径落盘，需双缓冲 + 低优先级任务）：Phase 1 已完成，见 [flight_log.md](flight_log.md)。
- [ ] 实测 SD 卡 / Flash 写入的 WCET 与掉电完整性（当前断电最多丢一个块 = 16 ms，且 Footer 缺失）。
- [ ] 在真实 Gazebo 上验证 1 kHz 时间契约与 UDP 传输抖动；`mock` 后端为锁步模型，不覆盖丢包/乱序/仿真时间不等比于墙钟的情况。
- [ ] 实测 `GazeboTransport` 后台任务在目标平台的执行时间与优先级，确认不饿死控制任务。
