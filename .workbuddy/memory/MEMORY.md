# HomeFly 项目长期记忆

## 项目定位
面向微型无人机的自主飞控系统（C11 + C++20，CMake+Ninja）。核心目标：软件与硬件解耦、平台无关、可仿真。
当前阶段 Phase 1 架构闭环已完成并验证（Logging / Blackbox `.hfl` / Gazebo SITL mock / WSL2 部署）。

## 关键方向（用户意图）
- **用户明确要以 WiFi 控制飞行**（不是传统 RC/SBUS）。即通信层需补「上行指令链路」：地面站经 WiFi(UDP) 下发 setpoint/arm/disarm/模式切换。
- 这映射到 `ILink` 抽象 + 一个 uplink command receiver；控制循环只读取最新 setpoint，网络 I/O 必须放在低优先级任务（与 Gazebo SITL 同构），保持 step() 无阻塞/无分配。
- 安全硬要求：链路丢失 → 触发 Failsafe/Disarm（类比 SafetyMonitor 超时）。

## 架构铁律（来自 SKILL.md，改代码必须遵守）
- FlightCore 不得依赖具体 MCU SDK/驱动；HAL 先抽象；文件名全小写 snake_case。
- 统一 `HF_LOG_*`；飞行日志真实落盘可离线校验；SITL 只经 SimulationSource + IImu/IMotor/IClock 适配器接入。

## 当前最大缺口（离真飞）
1. 飞行模式仅 Disarmed/Stabilize/Failsafe，缺 AltHold/PosHold/RTL/Auto。
2. 控制环仅 3 轴 angle+rate PID，缺高度/位置环、轨迹。
3. 通信仅遥测下行，缺 RC/指令上行、MAVLink。
4. 驱动几乎空白（仅 LED + 仿真 PWM）；esp32/stm32 平台仅 logging 占位，clock/timer/gpio/i2c/spi 未实现。
5. 传感器仅 IMU 在用，无气压计/磁力计/GPS、无 EKF 融合。
6. gz/classic SITL 已在 WSL2 Ubuntu + Gazebo Harmonic 8 真机验证（含 WSLg GUI 三终端闭环 PASS、原版 ArduPilot 机场 / 迷宫场景加载）；仍无 CI。

## WiFi 控制链路（已完成，2026-11-12）
- 28 字节帧 `control_protocol`（HFLC magic/FNV-1a/seq）+ `CommandReceiver`（poll 低优先级 / latest_setpoint 零 I/O）+ `UdpLink`（复用 gazebo UdpSocket）+ `tools/wifi_gcs.py`。
- 验证：CTest 16/16（含 test_wifi_integration 真实 UdpLink 环回集成测试：arm→stabilize→收敛 + 断臂断电）；二进制 e2e `--realtime --steps 13000 --wifi --link-timeout 500` + GCS `--rate 20 --count 300 --rearm-every 20` → **result: PASS**。
- 关键 gotcha（Windows 宿主）：
  1. LinuxTimer 需 timeBeginPeriod(1)（winmm）+ 尾部 400us 自旋，否则 ~15.6ms 调度 tick 周期性触发 ControlOverrun（reason=4）。
  2. --realtime 时控制线程 THREAD_PRIORITY_HIGHEST / 日志线程 BELOW_NORMAL（main.cpp），否则日志控制台 I/O 抢占控制线程一个时间片 → overrun。
  3. FSM 粘性 Failsafe 只能靠 DisarmRequest 离开（ArmRequest 仅 Disarmed 态生效，单元测试锁定语义不可改）；活链路下无超时 disarm，恢复靠 GCS 周期性 DISARM+ARM re-arm 手势。
  4. DISARM+ARM 背靠背两帧会被单次 poll() 合并 drain、互相抵消 → GCS 两帧间必须 sleep ≥150ms 分开消费。
  5. 主循环每步必须重新 `latest_setpoint()`（曾犯一次 pre-loop 捕获导致指令永远到不了控制器的集成 bug）。
  6. 非实时（虚拟时钟）模式二进制毫秒级跑完，UDP 帧 window 对不上——二进制 e2e 必须用 --realtime。
- 构建环境：PowerShell 跑 `tools/_build_ps.ps1`（cmake+ninja+ctest 重定向 build_logs/）；FC 输出在 Git Bash 直接跑可捕获，PowerShell `*>` 文件重定向捕获不到（自定义 log backend 缓冲）。

## 已验证环境
- MSVC 19.51 / CMake 4.3 / Ninja：16/16 CTest（test_wifi_integration 为真实 UDP 集成测试）。
- 本机 Git Bash 无 cmake/ninja/cl，需 `tools/build_windows_native.bat` 加载 MSVC。

## Git 推送（Windows 宿主，2026-11-13）
- 本机 PortableGit 的 **system gitconfig** 带 `credential.helper=helper-selector`：无交互会话里它会弹 GUI 选择器并挂死，表现为 `git push` 卡在 `git-credential-manager.exe get` 启动后无任何输出、数分钟后被 SIGTERM / timeout；但只读的 `git ls-remote` 一切正常，**极易误判成网络慢或 28MB 二进制太大**。
- 可用命令（跳过 system 配置，直连 `~/.gitconfig` 里的 GCM 缓存凭据，秒推）：
  `GIT_CONFIG_NOSYSTEM=1 GCM_INTERACTIVE=Never GIT_TERMINAL_PROMPT=0 git push -v origin main`
- 已验证无效：调大 `http.postBuffer`、后台运行、换 PowerShell 会话、清空 helper。别在这些方向上反复尝试；先 `git ls-remote` 与 `git rev-parse HEAD` 对比确认是否真的没推上去。
- 用户工作模式：Windows 侧 commit/push，WSL 侧 `git pull` 后自行验证；不要替用户跑 `git push`。

## WSLg GUI 部署约定（2026-11-13）
- 终端 1 一律用 `bash tools/run_wsl_gui.sh`：进程级 export `GZ_IP=127.0.0.1` + `GZ_SIM_RESOURCE_PATH`，自检 `gz` / world / `models/runway/model.config`，启动后自动调 `gz_focus_homefly.sh` 对焦。**不要手敲分两行 export** —— shell export 不跨终端，已踩坑 3 次卡在 `GUI requesting list of world names...`（真实卡点是 `/gazebo/worlds` 服务无响应；`server may be busy downloading resources` 是写死提示文本，不代表真在下载）。
- 终端 2 的 bridge 仍需自己 `export GZ_IP=127.0.0.1`；机场 world 必须让 `GZ_SIM_RESOURCE_PATH` 指向 `tools/gazebo/models`。
- 找不到飞机（0.3 m Iris vs 1500×100 m 跑道 + 固定默认相机 pose）→ `bash tools/gz_focus_homefly.sh`，打杆时加 `--follow`。**禁止**往 world SDF 加 `<gui>` 段：Gazebo Sim 会整体覆盖默认 GUI 配置导致窗口空白；持久化改用 `gz sim --gui-config <完整配置>`。
- 服务齐全但视口仍空白 → 终端 1 加 `export LIBGL_ALWAYS_SOFTWARE=1` 重启（WSLg 无 GPU 时 ogre2 + PBR 贴图可能起不来）。
