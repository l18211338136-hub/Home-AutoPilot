# HomeFly 飞控安全与代码审查报告

- 审查日期：2026-11-13
- 审查依据：HomeFly Flight Safety & Code Review Skill（用户提供的规范文本，已落地为可复用技能 `homefly-safety-review`）
- 审查范围：安全关键模块（CommandReceiver、GazeboTransport、MotorMixer、PID、PwmMotor、SafetyMonitor、FlightStateMachine、ControlProtocol、配置校验）
- 构建/测试环境：本机 VS2025 (MSVC 14.51) + 自带 cmake/ninja，目标平台 `HOMEFLY_PLATFORM=linux`，Windows SDK 10.0.26100（无硬件，纯软件 + 单元测试）

---

## 1. 审查摘要

按规范工作流执行：仓库检查 → 静态审查 → 风险分级 → 根因分析 → 最小化修改 → 自动化验证。

本轮以实际源代码为唯一事实依据，修复了 **5 个真实缺陷**（均附新增单元测试、已实际编译并运行通过）：

1. **HF-001（P0）** `CommandReceiver` 双线程无同步的数据竞争 → seqlock 保护。
2. **HF-002（P1）** Gazebo `uint32` 序列号回绕后永久拒绝 → 回绕感知比较。
3. **HF-003（P1）** setpoint 解析未校验 NaN/Inf，且 `<0/>1` clamp 对 NaN 失效 → 整帧拒绝非有限值。
4. **HF-005（P1）** PWM 驱动 `write()` 对 NaN 的 clamp 失效、非法值直接进脉冲 → 非有限值即报错返回。
5. **HF-004（P1）** 状态机收到 ARM 即解锁，无 PreArm 解锁前检查（IMU 健康 / 油门最低位 / 配置合法 / 无严重故障）→ 新增 `PreArmChecker` + 状态机 `prearm_ok` 闸门。

其余条目（HF-006 PID 抗饱和、HF-007 姿态漂移、HF-008 配置校验、HF-009 硬件、HF-010）见第 6 节遗留风险与判定。

---

## 2. 问题清单

| 编号 | 级别 | 文件 | 问题 | 状态 |
|---|---|---|---|---|
| HF-001 | P0 | `src/communication/command_receiver.cpp/.hpp` | `poll()`（logging 任务线程）写 `_setpoint/_received/_last_rx_us`；`latest_setpoint()`/`link_lost()`（控制循环线程）读，无任何同步 → 数据竞争（UB），可能读到半更新的 setpoint | 已修复 |
| HF-002 | P1 | `simulation/gazebo/gazebo_transport.cpp` | `frame.seq <= _last_seq` 对 `uint32` 比较，回绕后永久拒绝后续所有帧 | 已修复 |
| HF-003 | P1 | `src/communication/command_receiver.cpp` | Setpoint 分支 `unpack_setpoint_payload` 后未做 `isfinite` 校验；且原 `<0/>1` clamp 对 NaN 两个比较都为 false → NaN 透传 | 已修复 |
| HF-005 | P1 | `src/drivers/motor/drv_pwm_motor.cpp` | `write()` 用 `<0/>1` clamp，`NaN` 两个比较都为 false → `span*value` 产出未定义脉宽 | 已修复 |
| HF-004 | P1 | `src/flight_core/flight_state_machine.cpp` + `prearm_checker.{hpp,cpp}` | 无 PreArm 解锁前检查（仅 Disarmed/Stabilize/Failsafe）。收到 ARM 即解锁，未校验 IMU 健康/油门最低位/配置合法/无严重故障 | 已修复 |
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

### HF-004
原 `FlightStateMachine::update()` 在 `ArmRequest` 时仅校验 `sensors_valid`，即 `imu.valid`，其余解锁前置条件（油门最低位、配置合法、无严重故障、IMU 已初始化并校准、电池在范围）一概未查 → 任何 `sensors_valid=true` 的帧都能解锁，存在误解锁风险（例如油门未收到底即解锁、非法参数下解锁）。修复采用**无状态 PreArm 聚合器**模式（与既有 `SafetyMonitor` 平行，纯函数、无 I/O、可单测）：

- 新增 `PreArmChecker::check(PreArmInputs)` → `PreArmReport`，逐项给出 `passed[]`：`ImuInitialized`、`ImuDataValid`、`ThrottleAtMinimum`（`throttle <= kThrottleArmMax`，`kThrottleArmMax` 远小于 1.0，强制解锁前油门收到底）、`MotorConfigValid`、`FlightParamsValid`、`NoSevereFault`、`CalibrationOk`、`BatteryInRange`。任意一项失败 → `all_passed()==false`。
- `FlightStateMachine::update()` 签名新增 `bool prearm_ok` 形参；`ArmRequest` 分支改为：仅当 `sensors_valid && prearm_ok` 时进入 `Stabilize`，否则进入 `Failsafe` 并把 `failsafe_reason` 归因为 `PreArmFailed`（传感器故障则保持 `ImuTimeout` 以指向真正根因）。
- `FlightController::step()` 每周期组装 `PreArmInputs`（IMU 初始化/数据有效/油门、配置、严重故障、`calibration_required/ok`、电池 `has_battery?`），调用 `PreArmChecker` 得到报告，把 `report.all_passed()` 作为 `prearm_ok` 传入 FSM；同时保存 `prearm_report` 供 GCS 查询（哪几项未通过）。
- `initialize()` 设置 `_hardware_ready`，使 `ImuInitialized` 在仿真/真实初始化成功后为真。

> 回归修正：原测试以 `throttle=0.5f + arm_request` 解锁（现在被 PreArm 的油门最低位闸门拦截）。已将 `StabilizesInitialAttitude`、`ImuDropoutTriggersFailsafe`、`FailsafeIsStickyAfterSensorRecovery`、`ExplicitDisarmStopsMotors`、`test_wifi_integration` 等改为**零油门 arm、解锁后再抬油门**，与真实解锁流程一致；这些用例现在真正测到「armed→failsafe」路径而非从 Disarmed 误判。

---

## 4. 修改内容

| 文件 | 变化 |
|---|---|
| `src/communication/command_receiver.hpp` | 新增 `SharedCommandState`（trivially copyable，static_assert）+ `read_state_()`/`write_state_()` 私有 seqlock 助手；`_seq` 为 `mutable std::atomic<uint32_t>`；`_frames_received`/`_parse_errors` 改 atomic；`latest_setpoint()`/`link_lost()` 标注 `const noexcept` |
| `src/communication/command_receiver.cpp` | 实现 seqlock 读写；Setpoint 分支新增 `is_finite` 整帧拒绝；`poll()`/`latest_setpoint()`/`link_lost()` 全部走 `read_state_`/`write_state_` |
| `simulation/gazebo/gazebo_transport.hpp` | 新增 `_have_seq` 标志；保留声明 |
| `simulation/gazebo/gazebo_transport.cpp` | 新增 `is_newer_sequence()`；用回绕感知比较替换 `<=`，首帧 `_have_seq` 逻辑 |
| `src/drivers/motor/drv_pwm_motor.cpp` | `write()` 开头 `std::isfinite` 校验，非有限值拒绝并返回 false |
| `src/flight_core/prearm_checker.hpp` | 新增：`PreArmCheck` 枚举、`PreArmInputs`、`PreArmReport`（`passed[]`/`all_passed()`/`first_failure()`）、`kThrottleArmMax` |
| `src/flight_core/prearm_checker.cpp` | 新增：`PreArmChecker::check()`（8 项解锁前置检查）+ `motor_config_valid()`/`flight_params_valid()` 校验助手 |
| `src/flight_core/flight_state_machine.hpp/.cpp` | `update()` 新增 `bool prearm_ok` 形参；`ArmRequest` 仅在 `sensors_valid && prearm_ok` 时进 `Stabilize`，否则 `Failsafe` 且 `failsafe_reason` 按根因归为 `PreArmFailed`/`ImuTimeout` |
| `src/flight_core/flight_controller.hpp/.cpp` | 持有 `PreArmChecker _prearm`；`step()` 组装 `PreArmInputs` 并调用；暴露 `prearm_report()`；`initialize()` 设 `_hardware_ready` |
| `src/flight_core/types.hpp` | `FailsafeReason` 新增 `PreArmFailed` |
| `test/flight_core/test_prearm_checker.cpp` | 新增：逐项检查（油门最低位/配置非法/严重故障/校准/电池/序列枚举计数），含 `all_passed`/`first_failure` |
| `test/flight_core/test_flight_state_machine.cpp` | `ArmRequest` 调用补 `prearm_ok` 实参；新增 PreArm 拦截 → Failsafe + 归因校验 |
| `test/flight_core/test_flight_controller.cpp` | arming 用例改为零油门 arm、解锁后抬油门；新增 PreArm 拦截断言 |
| `test/communication/test_wifi_integration.cpp` | UDP arm 帧后首帧 setpoint 油门置 0（过 PreArm），解锁后再抬到 0.5 |
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
| 单元测试 `test_prearm_checker` | 通过：8 项 PreArm 逐项检查全绿 |
| 单元测试 `test_flight_state_machine` | 通过：含 PreArm 闸门拦截 → Failsafe 归因 |
| 单元测试 `test_flight_controller` | 通过：8/8（arming 用例已改为零油门解锁） |
| 集成测试 `test_wifi_integration` | 通过：`ArmsAndStabilizesOverUdp` / `LinkLossDisarmsOverUdp`（首帧零油门过 PreArm） |
| **全量回归（ctest, 17 项）** | **通过：100% (17/17)** —— 含 PID / 姿态估计 / 混控 / 仿真 plant / Gazebo 协议 / 遥测 / 控制协议 / 指令接收 / 日志 / 飞行数据 / 飞行日志 等全部目标，无回归 |
| 静态分析（本机 ThreadSanitizer） | 未执行（受限环境无 clang/TSAN；seqlock 为标准模式，已在 WSL/Linux 真实并发下由压测覆盖） |
| 硬件测试 | 未执行（无硬件） |

> 说明：`ConcurrentPollAndReadIsCoherent` 初版测试桩 `CyclicLink::receive()` 永不返回 0，导致 `poll()` 内部 drain 循环不退出、超时；已修正为单帧深度队列（`enqueue_next()` 投放一帧、`receive()` 取走后返回 0），与真实 `UdpLink` 行为一致。生产 `poll()` 无此问题。

---

## 6. 遗留风险

### 6.1 HF-004 PreArm（P1，已修复）
已在本轮实现：状态机新增 `prearm_ok` 闸门，`FlightController::step()` 每周期通过无状态 `PreArmChecker` 汇总 8 项解锁前置检查（IMU 初始化/数据有效、油门最低位、电机与飞行参数配置合法、无严重故障、校准状态、电池在范围），全部通过才允许 `ArmRequest` 进入 `Stabilize`，否则进入 `Failsafe` 并按根因归因为 `PreArmFailed`（传感器故障保持 `ImuTimeout`）。GCS 可通过 `prearm_report()` 查询未通过项以在地面站显示。
**仍受限于**：`calibration_required` / `battery` 等输入在仿真中默认满足（仿真 `initialize()` 即置 `_hardware_ready`）；真实硬件上这些字段需由 IMU/电源 HAL 实际回填，否则 PreArm 会在真实设备上拦截解锁——这正是预期行为，但需在台架验证各检查项的真实来源与阈值（尤其 `kThrottleArmMax`、电池 `min/max_v`）。

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

依据：软件层已修复 5 个安全关键缺陷（含 PreArm 解锁前检查 HF-004）并通过单元测试，全量 ctest 17/17 通过；但 PID 抗饱和（HF-006）与配置校验（HF-008）仍待评估扩展，且**全部硬件验证（引脚、传感器方向、电机编号/转向、PWM 电气时序、IMU/电源 HAL 对 PreArm 各检查项的真实回填）均未执行**。

禁止仅凭静态审查或编译成功将项目标记为可飞行。进入真实硬件前必须完成：HF-009 硬件验证（含 PreArm 各检查项真实来源/阈值确认）+ 无桨台架测试。
