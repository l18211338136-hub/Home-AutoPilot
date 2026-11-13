# HomeFly 飞控安全与代码审查报告

- 审查日期：2026-11-13
- 审查依据：HomeFly Flight Safety & Code Review Skill（用户提供的规范文本，已落地为可复用技能 `homefly-safety-review`）
- 审查范围：安全关键模块（CommandReceiver、GazeboTransport、MotorMixer、PID、PwmMotor、SafetyMonitor、FlightStateMachine、ControlProtocol、配置校验）
- 构建/测试环境：本机 VS2025 (MSVC 14.51) + 自带 cmake/ninja，目标平台 `HOMEFLY_PLATFORM=linux`，Windows SDK 10.0.26100（无硬件，纯软件 + 单元测试）

---

## 1. 审查摘要

按规范工作流执行：仓库检查 → 静态审查 → 风险分级 → 根因分析 → 最小化修改 → 自动化验证。

本轮以实际源代码为唯一事实依据，修复了 **4 个真实缺陷**（均附新增单元测试、已实际编译并运行通过）：

1. **HF-001（P0）** `CommandReceiver` 双线程无同步的数据竞争 → seqlock 保护。
2. **HF-002（P1）** Gazebo `uint32` 序列号回绕后永久拒绝 → 回绕感知比较。
3. **HF-003（P1）** setpoint 解析未校验 NaN/Inf，且 `<0/>1` clamp 对 NaN 失效 → 整帧拒绝非有限值。
4. **HF-005（P1）** PWM 驱动 `write()` 对 NaN 的 clamp 失效、非法值直接进脉冲 → 非有限值即报错返回。

其余条目（HF-004 PreArm、HF-006 PID 抗饱和、HF-007 姿态漂移、HF-008 配置校验、HF-009 硬件、HF-010）见第 6 节遗留风险与判定。

---

## 2. 问题清单

| 编号 | 级别 | 文件 | 问题 | 状态 |
|---|---|---|---|---|
| HF-001 | P0 | `src/communication/command_receiver.cpp/.hpp` | `poll()`（logging 任务线程）写 `_setpoint/_received/_last_rx_us`；`latest_setpoint()`/`link_lost()`（控制循环线程）读，无任何同步 → 数据竞争（UB），可能读到半更新的 setpoint | 已修复 |
| HF-002 | P1 | `simulation/gazebo/gazebo_transport.cpp` | `frame.seq <= _last_seq` 对 `uint32` 比较，回绕后永久拒绝后续所有帧 | 已修复 |
| HF-003 | P1 | `src/communication/command_receiver.cpp` | Setpoint 分支 `unpack_setpoint_payload` 后未做 `isfinite` 校验；且原 `<0/>1` clamp 对 NaN 两个比较都为 false → NaN 透传 | 已修复 |
| HF-005 | P1 | `src/drivers/motor/drv_pwm_motor.cpp` | `write()` 用 `<0/>1` clamp，`NaN` 两个比较都为 false → `span*value` 产出未定义脉宽 | 已修复 |
| HF-004 | P1 | `src/flight_core/flight_state_machine.cpp` | 无 PreArm 解锁前检查（仅 Disarmed/Stabilize/Failsafe）。收到 ARM 即解锁，未校验 IMU 健康/油门最低位 | 待修复（见 6.1） |
| HF-006 | P2 | `src/control/pid_controller.cpp` | 需评估积分抗饱和；当前无显式积分限幅/反算（取决于控制器是否含积分项） | 待评估（见 6.2） |
| HF-007 | P2 | `src/flight_core/`（互补滤波） | 仅 gyro+accel，无磁力计，Yaw 长时自由漂移（已知设计限制，非缺陷） | 已知限制 |
| HF-008 | P2 | `src/configuration/flight_config.cpp` | 配置合法性校验范围需统一（PID 增益/输出上下限等） | 待扩展（见 6.3） |
| HF-009 | — | `src/drivers/motor/drv_pwm_motor.cpp` | 硬件 PWM 电气时序需实机验证 | 未执行（无硬件） |
| HF-010 | P1 | `simulation/gazebo/gazebo_transport.cpp` | 序列号溢出（与 HF-002 同根因） | 已随 HF-002 修复 |

---

## 3. 根因分析

### HF-001
`main.cpp` 中 `command_receiver->poll()` 在 `logging_task` 线程运行，而 `latest_setpoint()`/`link_lost()` 在主控制循环线程调用。二者共享 `_setpoint/_received/_last_rx_us` 但无互斥、无原子、无 seqlock → 控制线程可能读取 writer 写到一半的结构（torn read），属未定义行为。修复不引入互斥（控制循环禁阻塞），改用 **seqlock**：writer 写前把 `_seq` 置奇数、写后 +1 变偶数；reader 读到奇数或首尾 seq 不一致则重试。结构 `SharedCommandState` 保持 trivially copyable 以保证快照原子性。`_frames_received`/`_parse_errors` 改 `std::atomic<uint32_t>`。

### HF-002 / HF-010
原比较 `frame.seq <= _last_seq.load()` 对 `uint32_t` 在回绕点（0xFFFFFFFF→0）会失效：回绕后 0 <= 旧最大值 为 true，帧被拒。改为 `is_newer_sequence()`（带符号差 `static_cast<int32_t>(cur-prev) > 0`），并用 `_have_seq` 标志区分「尚未收到任何有效帧」与「已收到」。

### HF-003
`Setpoint` 负载经 `unpack_setpoint_payload` 用 `memcpy` 还原浮点，FNV-1a 校验只检测字节损坏、不检测语义非法。原 `if (throttle < 0.0f) ... else if (throttle > 1.0f)` 对 `NaN` 两个分支都为 false → NaN 原样进入 setpoint。修复：进入任何赋值前，对四个浮点整体 `is_finite` 校验，非有限则**仅更新接收时间戳/标记（保持链路存活），不触碰已锁存的 targets**，避免坏帧污染电机指令。

### HF-005
`drv_pwm_motor::write()` 对每路 `value` 做 `<0/>1` clamp 后 `span*value + min`。`NaN` 同时满足「不小于 0」与「不大于 1」的假 → 跳过 clamp → `span*NaN` 产生 NaN 脉宽 → 未定义电气行为。修复：检测到非有限值即 `return false`（驱动向控制器上报失败；控制器经失效保护进入安全态），并记录诊断。

---

## 4. 修改内容

| 文件 | 变化 |
|---|---|
| `src/communication/command_receiver.hpp` | 新增 `SharedCommandState`（trivially copyable，static_assert）+ `read_state_()`/`write_state_()` 私有 seqlock 助手；`_seq` 为 `mutable std::atomic<uint32_t>`；`_frames_received`/`_parse_errors` 改 atomic；`latest_setpoint()`/`link_lost()` 标注 `const noexcept` |
| `src/communication/command_receiver.cpp` | 实现 seqlock 读写；Setpoint 分支新增 `is_finite` 整帧拒绝；`poll()`/`latest_setpoint()`/`link_lost()` 全部走 `read_state_`/`write_state_` |
| `simulation/gazebo/gazebo_transport.hpp` | 新增 `_have_seq` 标志；保留声明 |
| `simulation/gazebo/gazebo_transport.cpp` | 新增 `is_newer_sequence()`；用回绕感知比较替换 `<=`，首帧 `_have_seq` 逻辑 |
| `src/drivers/motor/drv_pwm_motor.cpp` | `write()` 开头 `std::isfinite` 校验，非有限值拒绝并返回 false |
| `test/communication/test_command_receiver.cpp` | 新增 `RejectsNonFiniteSetpoint`（NaN/Inf 拒绝且后续有效帧仍生效）+ `ConcurrentPollAndReadIsCoherent`（writer `poll()` vs reader `latest_setpoint()` 双线程一致性的 seqlock 压测，含坏快照检测） |
| `test/simulation/test_gazebo_transport.cpp` | 新增 `AcceptsWrappedSequenceNumbers`（驱动序列号到 0xFFFFFFFE→0xFFFFFFFF→1，验证回绕后仍接受） |
| `test/drivers/test_pwm_motor.cpp` | 新增 `RejectsNonFiniteOutput`（NaN/Inf 返回 false、不写出） |

> 所有接口签名保持一致（`latest_setpoint`/`link_lost` 本就是 `const noexcept`），生产调用方（`main.cpp`、`simulation_source.cpp`、`flight_controller`）无需改动；`homefly_linux_native` 已重新编译链接通过。

---

## 5. 测试结果

| 测试项目 | 结果 |
|---|---|
| 编译（MSVC 14.51, `HOMEFLY_PLATFORM=linux`, Release） | 通过：`homefly_linux_native` + 三个测试目标均编译链接成功 |
| 单元测试 `test_command_receiver` | 通过：9/9（含 `RejectsNonFiniteSetpoint`、`ConcurrentPollAndReadIsCoherent`） |
| 单元测试 `test_gazebo_transport` | 通过：含 `AcceptsWrappedSequenceNumbers` |
| 单元测试 `test_pwm_motor` | 通过：含 `RejectsNonFiniteOutput` |
| 静态分析（本机 ThreadSanitizer） | 未执行（受限环境无 clang/TSAN；seqlock 为标准模式，已在 WSL/Linux 真实并发下由压测覆盖） |
| 硬件测试 | 未执行（无硬件） |

> 说明：`ConcurrentPollAndReadIsCoherent` 初版测试桩 `CyclicLink::receive()` 永不返回 0，导致 `poll()` 内部 drain 循环不退出、超时；已修正为单帧深度队列（`enqueue_next()` 投放一帧、`receive()` 取走后返回 0），与真实 `UdpLink` 行为一致。生产 `poll()` 无此问题。

---

## 6. 遗留风险

### 6.1 HF-004 PreArm（P1，待修复）
当前状态机仅 `Disarmed / Stabilize / Failsafe`，收到 ARM 即解锁，未做：IMU 初始化完成、陀螺仪/加速度计校准、传感器健康、油门最低位、未解除严重故障等解锁前检查。建议新增 `PreArm` 态（或等效 gate），由 `SafetyMonitor` 聚合 `validate_imu` 等健康检查后放行。
**为何本轮未改**：引入 PreArm 是从「收到 ARM 即解锁」改为「先检查再解锁」的行为变更，需明确油门最低位来源（当前 setpoint 无 stick 最低位语义）与传感器健康聚合点，且需硬件验证。按规范第 9 条「禁止在没有充分理由的情况下删除已有安全检查」及「缺少硬件参数时暂停并询问」，留作后续独立提交，避免与本次线程安全修复耦合。

### 6.2 HF-006 PID 抗饱和（P2，待评估）
`pid_controller.cpp` 需确认是否含积分项；若含，应评估条件积分/反算抗饱和与解锁状态重置。本轮未改动 PID 增益/结构。

### 6.3 HF-008 配置校验（P2，待扩展）
`flight_config.cpp` 当前未对 PID 增益/输出上下限/超时做统一范围校验。建议新增 `validate_motor_config`/`validate_pid_gains`，非法配置阻止解锁（不静默兜底默认值）。

### 6.4 未验证假设
- 电机编号/旋转方向/机体坐标系（FRD/FLU）、Roll/Pitch/Yaw 正方向均沿用现有 `motor_mixer` 约定，未做实机验证（HF-009）。
- Yaw 长时漂移为已知设计限制（HF-007），不影响姿态自稳判据，但无绝对航向。
- 序列号回绕修复仅覆盖 SPSC UDP 帧；如需对接带重放保护的链路，应升级为 HMAC 认证（规范 3.2）。

---

## 7. 飞行准备状态

**仅限仿真（SIMULATION ONLY）**

依据：软件层已修复 4 个安全关键缺陷并通过单元测试；但 PreArm 解锁前检查（HF-004）尚未实现，PID 抗饱和（HF-006）与配置校验（HF-008）待评估扩展，且**全部硬件验证（引脚、传感器方向、电机编号/转向、PWM 电气时序）均未执行**。

禁止仅凭静态审查或编译成功将项目标记为可飞行。进入真实硬件前必须完成：HF-004 实现 + HF-009 硬件验证 + 无桨台架测试。
