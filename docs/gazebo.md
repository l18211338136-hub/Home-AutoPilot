# Gazebo SITL 接入

HomeFly 通过**外部仿真传输适配器**接 Gazebo：飞控侧不链接任何 Gazebo / protobuf / ZeroMQ 库，Gazebo 侧是一个独立桥接进程，两者之间用固定 64 字节 UDP 帧通信。切换仿真器不动 FlightCore、日志、遥测和飞行日志。

## 1. 为什么这样设计

| 约束 | 做法 |
| --- | --- |
| 不引入第三方依赖 | 线协议是纯 POD，飞控侧只用标准库 + Winsock/POSIX socket |
| 控制循环零 I/O | `IImu::read()` / `IMotor::write()` 只读写 SPSC 环，UDP 收发在独立低优先级任务 |
| 平台可移植 | `UdpSocket` 把 Winsock / POSIX 差异关在 `udp_socket.cpp`；MCU 上可整体替换为其它传输 |
| 业务模块零改动 | `SimulationSource` 是唯一注入点，主循环对内置 plant 与 Gazebo 完全相同 |
| 时间语义正确 | `GazeboClock` 由「被消费的 IMU 帧时间戳」推进，不用宿主墙钟，IMU 超时校验不会误判 |

```
Gazebo / mock plant                     HomeFly (Linux Native / MCU)
┌───────────────────────────┐  UDP   ┌──────────────────────────────────┐
│ tools/gazebo_bridge.py    │  IMU   │ GazeboTransport (低优先级任务)    │
│  IMU  -> HF_IMU 帧        │ ─────> │   └─> SPSC 环 -> GazeboImu:IImu   │
│  MOTOR <- HF_MOTOR 帧     │ <───── │   <─- SPSC 环 <- GazeboMotor:IMotor│
│  (gz / classic / mock)    │  MOTOR │ GazeboClock:IClock                │
└───────────────────────────┘        │ FlightController / 日志 / .hfl    │
                                      └──────────────────────────────────┘
```

## 2. 线协议（version 1）

固定 **64 字节、小端、自然对齐**，Python：`struct.Struct("<IIQIBBH8fII")`。

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | u32 | `magic` | `0x48465A47` = `"HFGZ"` |
| 4 | u32 | `seq` | 每个发送方严格递增，接收方按 seq 去重 |
| 8 | u64 | `timestamp_us` | Gazebo 仿真时间（微秒） |
| 16 | u32 | `checksum` | FNV-1a，覆盖全帧但跳过 `[16,20)` |
| 20 | u8 | `version` | 1 |
| 21 | u8 | `type` | 1=IMU, 2=MOTOR, 3=HELLO, 4=SHUTDOWN |
| 22 | u16 | `flags` | 保留 |
| 24 | f32[8] | `values` | IMU: `[0..2]` gyro rad/s、`[3..5]` accel m/s²、`[6]` 温度；MOTOR: `[0..3]` 归一化油门 |
| 56 | u32 | `reserved` | 保留 |
| 60 | u32 | `tail` | 0 |

校验拒绝：错误 magic / version / type / checksum / 非有限浮点值。`test/simulation/test_gazebo_protocol.cpp` 内置由 Python 生成的 golden 帧，逐字节验证两种语言实现一致。

## 3. 本机快速验证（不需要 Gazebo）

> Windows 宿主想接真实 Gazebo，先看[第 8 节 WSL2 部署](#8-wsl2--ubuntu-部署windows-宿主的推荐路径)：三件套必须全部跑在 Linux 侧，Windows 的 `.exe` 不能与 WSL2 桥接直接走 UDP 回环。

`mock` 后端用纯 Python 复刻 `simulation/sim_plant.cpp` 的刚体姿态模型，与飞控锁步交换数据：发一帧 IMU → 等一帧电机 → 推进物理 → 发下一帧 IMU。**先启动桥接或先启动飞控都可以**（桥接重发同一 seq，飞控按 seq 去重）。

终端 A：

```bash
python tools/gazebo_bridge.py --backend mock --steps 2000
```

终端 B：

```bash
./build/linux/src/applications/linux_native/homefly_linux_native \
    --steps 2000 --gazebo --flight-log build/linux/gazebo_sitl.hfl
```

Windows 上一条命令跑完（Git Bash）：

```bash
PY=/c/Users/admin/.workbuddy/binaries/python/versions/3.13.12/python.exe
"$PY" tools/gazebo_bridge.py --backend mock --steps 2000 > /tmp/bridge.log 2>&1 &
sleep 0.3
./build/linux/src/applications/linux_native/homefly_linux_native.exe --steps 2000 --gazebo
wait
```

协议自测（无网络）：

```bash
python tools/gazebo_bridge.py --self-test
```

失效保护场景：两端 `--steps 1000`，飞控加 `--failsafe`。

## 4. 接真实 Gazebo Garden / Harmonic

### 4.1 依赖与话题

桥接的 `gz` 后端动态探测 `gz.transport*` / `gz.msgs*`（无需编译 C++ 插件）：

```bash
pip install gz-transport-python gz-msgs-python   # 或使用发行版自带绑定
python tools/gazebo_bridge.py --backend gz \
    --imu-topic /world/quad/model/homefly/link/body/sensor/imu/imu \
    --motor-topic /homefly/actuators
```

`tools/gazebo/homefly_quad.world` 是完整可加载的 world（1 ms / 1 kHz 物理步长、地面、4 个 rotor link + revolute joint、IMU 传感器与 4 个 `MulticopterMotorModel` 插件），可直接 `gz sim -s -v4 tools/gazebo/homefly_quad.world`；`tools/gazebo/homefly_quad.sdf` 仍是裸 `<model>` 片段，供你嵌进自己的 world。插件从 `gz.msgs.Actuators.velocity` 取电机转速（rad/s），桥接把 `[0,1]` 油门乘 `--max-rot-velocity`（默认 1000）后填入，按插件的 `<motorNumber>` 索引。

> **电机话题对齐（踩过的坑）**：`MulticopterMotorModel` 订阅的是 `AsValidTopic(robotNamespace + "/" + commandSubTopic)`，而 `AsValidTopic` 会把 `//` 的**两个**斜杠一起删掉、且**不会补前导斜杠**。旧配置 `<robotNamespace>homefly</>` + `<commandSubTopic>/actuators</>` 被拼成 `homefly//actuators` → `homeflyactuators`，桥接发布的 `/actuators` 永远到不了电机插件。现在 world 不设 `robotNamespace`，gz 回退用模型名 `homefly`，插件订阅 `homefly/actuators`（gz-transport 规范化为 `/homefly/actuators`），与桥接默认 `--motor-topic` 一致。**改模型名必须同步改 `--motor-topic`**。首次运行仍请按发行版核对插件文件名、IMU `<topic>`（默认含 world 名 `quad`）与机型参数。

> **Gazebo Sim 8 (Harmonic) 必读**：IMU 传感器由独立的 `gz-sim-imu-system`（`gz::sim::systems::Imu`）创建并发布，`gz-sim-sensors-system` 只管 camera/lidar 等渲染型传感器。world 里必须同时声明两者，否则 `gz topic -l` 里不会出现任何 IMU 话题、桥接 `imu_frames=0`。日志里的 `[Sensors.cc] Waiting for init` 是渲染线程的正常等待，与 IMU 无关。此外 `-s` 启动的 server 默认暂停，必须加 `-r` 才会推进仿真并产生数据。

### 4.2 轴系与单位

| 项 | HomeFly 约定 | Gazebo 默认 | 处理 |
| --- | --- | --- | --- |
| 坐标系 | body，右手，z 向上 | FLU：x 前、y 左、z 上 | 直接映射；若模型用 FRD，用 `--axis-map` / `--axis-sign` 调整 |
| gyro | rad/s | `angular_velocity` rad/s | 直通 |
| accel | m/s²，含重力比力 | `linear_acceleration` m/s² | 直通；姿态解算只用 accel+gyro，**不需要四元数输入** |
| 时间戳 | `timestamp_us`，单调递增 | `header.stamp` (sec+nsec) | 桥接转成 us；无 stamp 时按 `--period-us` 合成 |
| 电机 | 4 个 `[0,1]` | `Actuators.velocity` rad/s | 乘 `max_rot_velocity` |

### 4.3 必须满足的时间契约

- `--period-us` 必须等于 `FlightConfig::control_period_us`（默认 1000）。Gazebo 的 IMU 传感器更新率要设为 1 kHz，或让桥接按 `--period-us` 合成时间戳。
- 每个控制周期**恰好一帧 IMU、一帧电机**。帧率不匹配会触发 `ControlOverrun` 或 IMU 超时；`GazeboTransport` 的统计（`imu_rx / imu_bad / imu_drop / motor_tx / motor_drop`）会指出是哪一侧出问题。
- 飞行日志的时间戳来自 Gazebo 仿真时间，因此 `.hfl` 的 `duration` 是仿真时长，不是墙钟时长。

### 4.4 Gazebo Classic 11

`--backend classic` 走 `pygazebo`（`pip install pygazebo protobuf`）。Classic 的话题名、消息字段和订阅写法随版本变化较大，桥接里预留了接线点，需按你的 Gazebo 版本补齐 `publish_actuators()` / 订阅回调；协议与飞控侧完全不变。

## 5. 参数

飞控侧（`--help`）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--gazebo` | 关 | 启用外部 SITL，替换内置 SimPlant |
| `--steps N` | 2000 | 最多处理 N 帧 IMU |
| `--gazebo-host IP` | `127.0.0.1` | 桥接所在主机 |
| `--gazebo-local-port N` | 14550 | 飞控收 IMU 的端口 |
| `--gazebo-remote-port N` | 14551 | 飞控发电机的端口 |
| `--gazebo-timeout MS` | 3000 | 等下一帧 IMU 的超时，超时即结束并判 `FAIL` |
| `--flight-log PATH` | `homefly_flight.hfl` | SITL 同样生成可离线校验的 `.hfl` |
| `--realtime` | — | 与 `--gazebo` 互斥（Gazebo 拥有仿真时间），同时给出会被忽略并告警 |

桥接侧（`python tools/gazebo_bridge.py --help`）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--backend` | `mock` | `mock` / `gz` / `classic` |
| `--host` / `--imu-port` / `--motor-port` | `127.0.0.1` / 14550 / 14551 | 与飞控端口镜像对应 |
| `--steps` | 2000 | 交换帧数（gz 模式 0 = 不限） |
| `--period-us` | 1000 | 仿真时间步长 |
| `--timeout` | 5.0 | 无进展超时（秒） |
| `--initial-roll` / `--initial-pitch` | 0.20 / -0.12 | mock 初始姿态（rad） |
| `--max-rot-velocity` | 1000 | 油门 → `Actuators.velocity` 的比例 |
| `--axis-map` / `--axis-sign` | `0,1,2` / `1,1,1` | 轴重排与符号 |

## 6. 故障排查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `GAZEBO` 日志 `imu_rx=0` 且 `result: FAIL` | 端口不一致 / 桥接未启动 | 核对镜像关系：桥接 `--imu-port` = 飞控 `--gazebo-local-port`，桥接 `--motor-port` = 飞控 `--gazebo-remote-port` |
| 桥接 `no motor frame for N s` | 飞控没收到 IMU，或最后一帧电机被吞 | 检查防火墙、端口；确认两端 `--steps` 一致 |
| `imu_bad` 持续增长 | 校验和/版本不匹配，或重复帧 | 确认桥接与飞控协议版本一致；少量重复帧是启动期正常重发 |
| `motor_drop` 增长 | 后台任务来不及发 | 检查宿主机负载；真机由低优先级任务在空闲窗口消费 |
| 立刻 `ControlOverrun` | `--period-us` 与 `control_period_us` 不一致 | 统一为 1000 us |
| 立刻 `ImuTimeout` | IMU 时间戳不是单调递增的仿真时间 | 让桥接用 Gazebo `header.stamp` 或合成时间戳 |

## 7. 已验证与边界

**已在本机端到端验证**（MSVC 19.51 / Windows / mock 桥接，以及 WSL2 Ubuntu 22.04 + Gazebo Harmonic 8.15.0 真实 gz 后端）：

- 真实 Gazebo Harmonic 8.15.0（WSL2 Ubuntu 22.04）：`bash tools/run_wsl_sitl.sh --backend gz` → `SITL run PASS`；`imu_rx=2001 imu_bad=0 imu_drop=0 motor_tx=2000 motor_drop=0`，`mode=stabilize` 全程保持，`.hfl --strict` 校验 PASS。桥接以锁步方式转发（每周期一帧 IMU，seq 合成 1kHz 时间戳）。
- `--gazebo` 正常 2000 帧：`result: PASS`，`imu_rx=2000 motor_tx=2000 imu_drop=0 motor_drop=0`；最终 roll `-0.0047` / pitch `0.0029` rad，与内置 SimPlant 场景一致。
- `--gazebo --failsafe` 1000 帧：`Failsafe` + `ImuReadFault` + 电机归零，`result: PASS`，桥接同样锁步跑满 1000 帧。
- SITL 生成的 `.hfl` 经 `hfl_dump.py --strict` 校验 `PASS`，可导出 CSV。
- 13 / 13 CTest 通过，含协议 golden 向量、UDP 回环、seq 去重、超时、HAL 互锁与 SPSC 环。
- `tools/run_wsl_sitl.sh` 在 Windows Git Bash 下以 mock 后端完整跑通：自动协议自测 → 桥接 → 飞控 2000 帧 → `.hfl --strict` 校验，脚本退出 `PASS`（脚本会自动探测 Windows 的 `.exe`，便于在未装 WSL 的主机上先验证流程）。
- `tools/gazebo/homefly_quad.world` 与 `.sdf` 均通过 XML 良构性解析（此前两个文件注释里的 `--backend` 会让 XML 注释非法，已修正）。
- **WSLg GUI 三终端全闭环**（Gazebo Sim GUI + bridge + FC，`--steps 30000`）：`mode=stabilize` 全程、`motors_zero=no`、`result: PASS`；`imu_rx=30000 imu_bad=0 imu_drop=0 motor_tx=30000 motor_drop=0 send_fail=0`，`.hfl` `records=30000 blocks=1875 footer=ok checksum=0x07AE67D2`。GUI 视口确认四个 rotor 由 FC 电机输出驱动、模型姿态响应，最终 roll/pitch 收敛到 0。

**尚未验证 / 预留**：

- `classic` 后端依赖真实 Gazebo Classic 及其 Python 绑定，本机未安装，需实测接线与消息字段。
- 当前是单机 UDP（`127.0.0.1`）；跨机部署需要处理丢包、乱序与重传，协议已带 `seq` 与校验和，但传输层尚未做可靠性保证。
- 当前模型只有姿态、无高度 / 位置 / 气动 / 传感器噪声与延迟；`docs/realtime_contract.md` 中列出的 WCET、SD 卡掉电完整性等真机指标仍需实测。

## 8. WSL2 / Ubuntu 部署（Windows 宿主的推荐路径）

### 8.1 平台怎么选

| 路径 | 官方支持 | 结论 |
| --- | --- | --- |
| Ubuntu（物理机 / WSL2） | apt 官方二进制，主平台 | **推荐**：插件、SDF、示例最全 |
| Windows 原生（Jetty + Pixi / conda-forge） | Experimental，官方文档注明 known runtime issues；Harmonic 写明 Conda-Forge 支持 not fully functional | 备选，需实测 |
| Gazebo Classic 11 | 无官方 Windows 二进制 | 勿选 |

### 8.2 为什么必须「三件套全在 Linux 侧」

WSL2 NAT 模式的 `localhostForwarding=true` **只转发 TCP**，UDP 的 `127.0.0.1` 跨 Windows↔WSL2 边界不通（microsoft/WSL#8868、#8783；`netsh portproxy` 同样只支持 TCP）。本项目的 SITL 全是 UDP 帧，因此：

- 「Windows 的 `homefly_linux_native.exe` + WSL2 的桥接 / Gazebo + 两边都填 `127.0.0.1`」**必然超时失败**。WSL interop 能启动 `.exe`，但它仍是 Windows 进程，UDP 命名空间在宿主侧。
- 正确做法：Gazebo、`gazebo_bridge.py`、飞控**都在 WSL2 里**，飞控用 CMake/GCC 重新编译成 Linux 原生 ELF。三者同一网络命名空间，`--host` / `--gazebo-host` 保持默认 `127.0.0.1` 即可。
- 若确实要混跑，必须双向显式 IP：桥接 `--host <Windows 主机 IP>`、飞控 `--gazebo-host <WSL2 IP>`，并放行 Windows 防火墙 UDP 14550 入站与 Hyper-V 防火墙 14551；飞控已绑 `INADDR_ANY`，可行但脆弱，不推荐。

### 8.3 一次性准备（WSL2 Ubuntu 终端）

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build python3 python3-pip git
# 按 Gazebo 官方页配 OSRF apt 源后安装：
sudo apt install -y gz-harmonic       # Ubuntu 22.04；24.04 用 gz-jetty

# 源码放 WSL2 的 ext4，别直接从 /mnt/d 编译（DrvFS 小文件 I/O 慢、权限/换行问题）
cd ~
cp -r /mnt/d/Github/Home-Fly ~/Home-Fly     # 或 git clone
cd ~/Home-Fly
```

> CMake 已通过 `find_package(Threads REQUIRED)` + `Threads::Threads` 显式链接线程库，`std::thread`（日志任务 / GazeboTransport / LinuxTimer）在 Linux 工具链下不会再出现 `undefined reference to pthread_create`。

### 8.4 一条命令跑通

```bash
bash tools/run_wsl_sitl.sh                          # 自动构建 + mock 2000 帧 + 校验 .hfl
bash tools/run_wsl_sitl.sh --steps 1000 --failsafe  # 失效保护场景
bash tools/run_wsl_sitl.sh --backend gz             # 真 Gazebo，headless
bash tools/run_wsl_sitl.sh --build                  # 强制原生重编译
```

脚本依次：协议 `--self-test` →（缺二进制时）`cmake -G Ninja` 构建 → 后台起 `gz sim -s`（仅非 mock）→ 后台起桥接 → 前台起飞控 `--gazebo` → 退出时清理后台进程 → `hfl_dump.py --strict` 校验 → 汇总 `PASS/FAIL`。产物在 `build/wsl/sitl/`：`bridge.log`、`gz.log`、`sitl.hfl`。

| 参数 / 环境变量 | 默认 | 说明 |
| --- | --- | --- |
| `--backend mock\|gz\|classic` | `mock` | 仿真源 |
| `--steps N` | 2000 | 两端统一的 IMU / 电机帧数 |
| `--world PATH` | `tools/gazebo/homefly_quad.world` | `--backend gz` 的 world |
| `--build` / `--no-build` | 自动 | 强制构建 / 只用已有二进制 |
| `--failsafe` | 关 | 40% 处丢 IMU，验证保护 |
| `BUILD_DIR` | `build/wsl` | 构建与产物目录（建议相对路径） |
| `PYTHON` | `python3` | 桥接 / 日志解析用的解释器 |

脚本内所有路径均为相对路径，因此在 Git Bash 下用 Windows 原生 `.exe` 做自测也不会被 MSYS 的路径转换破坏。

### 8.5 接真实 Gazebo 前的核对清单

1. `--backend gz` 需要 Python 绑定（`gz-transport-python` / `gz-msgs-python` 或发行版自带），缺失时桥接会给出安装提示；先用 `--backend mock` 确认飞控侧链路 OK。
2. **电机话题必须与 world 模型名一致**：插件订阅 `homefly/actuators`，桥接默认发布 `/homefly/actuators`；用 `gz topic -l | grep actuators` 和 server 日志里的 `Listening to topic:` 逐字核对。换模型名要同步改 `--motor-topic`。其余按发行版核对插件文件名、IMU `<topic>`、`maxRotVelocity` 与机型质量 / 惯量 / 臂长。
3. **pacing 已由桥接器保证**：`gz` 后端现在严格锁步——缓冲最新一帧 IMU，每个控制周期只发一帧，时间戳 `seq × period_us` 合成，不受 Gazebo 自身发布速率影响；启动阶段重复发首帧直到收到第一帧电机指令。早期「IMU 洪泛 → `imu_drop` → `ControlOverrun`」的问题已修复（`imu_rx` 与 `motor_tx` 应 1:1）。
4. 脚本退出时会 kill 记录的 Gazebo / 桥接 PID；若 Gazebo 派生了子进程，用 `pkill -f 'gz sim'` 手动收尾。
5. 桥接统计里的 `retransmits` 表示稳态等待窗口内没收到电机帧而重发同一 seq，飞控按 seq 去重，不影响结果；Windows 回环偶发抖时会增大并拉长墙钟耗时，仿真时间仍由帧时间戳驱动。

### 8.6 全 Windows 原生（备选）

Pixi + conda-forge 装 Gazebo Jetty，桥接与飞控都跑在 Windows，UDP 不出宿主，可完全绕开 8.2 的问题；用 `gz sim -s` headless 避开实验版 GUI / OGRE2 的已知缺陷。代价是该平台支持仍是实验性的，且 `gz` 后端的 Python 绑定齐全度未验证。细节以 [Gazebo 官方 Windows 安装页](https://gazebosim.org/docs/harmonic/install_windows) 为准。

### 8.7 WSLg GUI 可视化（三终端闭环）

headless `run_wsl_sitl.sh` 只验证了 **FC ↔ bridge** 的 UDP 闭环：即使 Gazebo 的
`MulticopterMotorModel` 一个 `Actuators` 都没收到，脚本照样会打印 `SITL run PASS`。
要看飞机真的被飞控驱动，必须开 GUI 并单独核对电机话题（见 8.7 末尾判据）。

WSLg 下 GUI 与 server 同进程启动（`gz sim -r`，**不要** `-s`）。GUI 卡在反复
`GUI requesting list of world names...` 是 gz-transport 组播发现问题，先设：

```bash
export GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE=1   # WSLg 无 GPU / ogre2 起不来时可选
```

三个终端，全部在 WSL 侧，先 `cd` 到仓库目录并 `git pull`：

```bash
# 终端 1：Gazebo GUI + server（-r 必须，否则暂停、无 IMU）
gz sim -r -v4 tools/gazebo/homefly_quad.world

# 终端 2：桥接（--steps 0 = 不限帧数，看完手动 Ctrl-C）
python3 tools/gazebo_bridge.py --backend gz --steps 0

# 终端 3：飞控（注意：FC 的 --steps 0 会立刻退出，必须给正整数）
./build/wsl/src/applications/linux_native/homefly_linux_native \
    --gazebo --steps 30000 --overrun-factor 10 --gazebo-timeout 10000
```

**启动顺序（比话题更容易踩）**：GUI 下 Gazebo 加载慢，physics 未推进时 IMU 会先发出
一帧 `linear_acceleration` 全零的样本。HomeFly 的 `validate_imu()` 要求
`accel_norm ∈ [0.5g, 2g]`，零帧会触发 `InvalidImuData`；而 Failsafe 是**粘性**的
（无 `--wifi` 时每周期只发 `ArmRequest`，该事件在 Failsafe 态无效，且没有
`DisarmRequest`），于是整个 run 锁死在 `mode=failsafe / motors_zero=yes`，即使链路
指标 `imu_rx` / `motor_tx` 全部跑满、`imu_drop=0` 也照样 `result: FAIL`。
桥接现已过滤物理上不合理的 IMU 帧、只锁存有效样本，但**必须先确认 Gazebo 在跑**
（GUI 里仿真时间在走、或按了播放），再启动 bridge 和 FC：

```bash
gz topic -e -t /world/quad/clock -n 2                                     # sim 时间应递增
gz topic -e -t /world/quad/model/homefly/link/body/sensor/imu/imu -n 2    # accel 约 9.81 m/s²
```

若 Gazebo 启动较慢、bridge 报 `no IMU sample`，加 `--timeout 20`（启动等待与空闲超时共用该值）。

**电机话题判据**（这是之前漏掉的一环）：

```bash
gz topic -l | grep actuators              # 应看到 /homefly/actuators
gz topic -e -t /homefly/actuators -n 3    # velocity[] 非零 = 桥接确实在发
```

再对照终端 1 日志里的 `Listening to topic:` 必须与桥接 advertise 的名字一致，
并在 GUI 视口里看到四个 rotor 随 FC 输出转动、机身姿态收敛。

两条日志无害，不必中断：`No robotNamespace set using entity name.` 是 gz 用模型名
兜底的正常警告；`Service call timed out for [/world/quad/gui/info]` 在 WSLg 下偶发，
后续 `MinimalScene` / `ogre2` 仍会继续加载。

> 想贴日志做可复现验证，把终端 2 也改成 `--steps 30000`，与 FC 同步结束。
> 想一直看就给 FC 一个更大的 `--steps`，终端 2 保持 `--steps 0`。

### 8.8 GUI + WiFi GCS 可视化遥控（四终端）

把 8.7 的 GUI 闭环和 WiFi GCS 合起来。多出来的坑是**两个时钟不同步**：

- FC 在 `--gazebo` 下用 Gazebo 合成时间戳作仿真时钟（`source->clock()`），
  `CommandReceiver` 用 `--link-timeout` 在这条**仿真时钟**上判断链路是否 stale
  （`command_receiver.cpp:39`、`:119`）。
- GCS 按**墙钟** 20 Hz 发帧；bridge 默认以 UDP 允许的最快速度锁步推进，30000 步
  可能在几秒墙钟内跑完，两帧 GCS 之间在仿真时间里已远超 500 ms，FC 会反复误判
  链路丢失并断臂。
- bridge 的 `--realtime` 把每帧 IMU 锚定到 `period_us`（1 ms）墙钟节拍，使
  **仿真时间 ≈ 墙钟时间**：GCS 帧与 FC 控制周期对齐，GUI 视口也才真正可实时观看。

> **同名的两个 `--realtime` 别搞混**：4.1 节表里那个是 **FC 参数**，与
> `--gazebo` 互斥、同时给会被忽略（所以终端 3 **不要**加）；这里的是
> **bridge 参数**，必须加在终端 2。两者作用对象完全不同。

四个终端，全部 WSL 侧，都先 `export GZ_IP=127.0.0.1`，`git pull` 后按序启动：

```bash
# 1 Gazebo GUI + server
gz sim -r -v4 tools/gazebo/homefly_quad.world

# 2 bridge（--realtime 必须，--steps 0 不限帧）
python3 tools/gazebo_bridge.py --backend gz --realtime --steps 0 --timeout 20

# 3 飞控（--gazebo 与 --wifi 可共存；link-timeout 放宽到 2 s 抗 WSL2 抖动）
./build/wsl/src/applications/linux_native/homefly_linux_native \
    --gazebo --wifi --link-timeout 2000 --steps 30000 \
    --overrun-factor 10 --gazebo-timeout 10000

# 4 地面站（最后启动；--count 0 持续发送，--interactive 读 stdin）
python3 tools/wifi_gcs.py --rate 20 --count 0 --interactive --throttle 0.5
```

终端 4 里逐行输入即可实时改杆（弧度 / 归一化油门），每个值在下一个 20 Hz 帧生效：

```
roll 0.20        # 横滚目标 0.20 rad
pitch -0.10      # 俯仰目标
yaw 0.50         # 偏航角速度 rad/s
t 0.70           # 油门 0..1
arm / disarm     # 手动上锁 / 解锁
q                # 退出（自动发 DISARM）
```

**两个容易踩的判定细节**：

1. GCS 必须先发 ARM 才能进 `Stabilize`（无 `--wifi` 时 hardcoded 的 `arm_request`
   会被 `latest_setpoint()` 取代）。
2. `main.cpp:624-627` 的 `result: PASS` 要求**结束瞬间** `mode=Stabilize` 且
   `|roll|,|pitch| < 0.05 rad`。这是自稳回归判据、不是遥控判据：打杆期间最后一步
   姿态不为零就会报 `FAIL`，属正常。想拿 PASS 就在收尾前把杆回中
   （`roll 0` / `pitch 0` / `yaw 0`）等姿态收敛，并**等 FC 打印 `result` 之后
   再在 GCS 输 `q`**，否则最后的 DISARM 会让 mode 变成 `Disarmed` 而 FAIL。
   想缩短一轮把终端 3 的 `--steps` 调小（10000 ≈ 10 s）。

**验收看这些，不要只看 `result`**：GUI 视口里飞机随打杆实时倾斜/加减速；
FC 日志 `frames_received` 持续增长、`link_lost=no`、`mode=stabilize`；
`gz topic -e -t /homefly/actuators -n 3` 的 velocity 随油门变化。

### 8.9 切换 ArduPilot 机场 / 迷宫场景与 Iris 风格外观

`tools/gazebo/` 下有四个可直接替换的 world，**桥接、飞控、GCS 命令一行都不用改**：

| world 文件 | 场景 | 来源 | 外部资源 |
| --- | --- | --- | --- |
| `homefly_quad.world` | 灰色空场（默认，已验证 PASS） | 本项目 | 无 |
| `homefly_maze.world` | 迷宫（14 面墙 + 深色地面） | `ArduPilot/ardupilot_gz` → `ardupilot_gz_gazebo/worlds/maze.sdf`，几何逐字复制 | 无 |
| `homefly_airport.world` | **逐字节原版** ArduPilot 机场（跑道 + `airfield.dae` 网格 + PBR 贴图） | `ArduPilot/ardupilot_gazebo` → `models/runway`，用 `<include><uri>model://runway</uri></include>` 加载 | `tools/gazebo/models/runway/`（上游完整目录，28 MB） |
| `homefly_airport_lite.world` | 同布局的轻量机场（跑道 / 中心线 / 阈值条 / 停机坪 / 机库 / 塔台 / 风向袋 / 油罐） | 按上游 `runway.sdf` 布局用内联 `<box>` 几何重建 | 无 |

**飞机外观统一是 3DR Iris 风格**：中心舱、红色顶盖、X 型四机臂、四个电机、
滑橇起落架与双叶桨标记，全部用内联几何拼出，不引用任何 mesh，因此不额外增加
资源路径依赖。改动只发生在 `<visual>` 元素 —— `inertial`、`collision`、
`link` / `joint` 名字、`sensor name="imu"` 及其话题、四个
`MulticopterMotorModel` 插件（`commandSubTopic=actuators`、无 `robotNamespace`）
**一个字都没动**，所以 8.7 / 8.8 的验证结论可直接迁移。若之后想换成原版
`iris.dae` 网格，只需替换这些 `<visual>`，物理与桥接仍不用碰。

四个文件都刻意保持 `world name="quad"`、`model name="homefly"`：桥接订阅
`/world/quad/model/homefly/link/body/sensor/imu/imu`，四个电机插件靠模型名兜底
订阅 `homefly/actuators`（规范化为 `/homefly/actuators`）。**改这两个名字就会断链**。
四个 `homefly` 模型是同一份拷贝（physics 1 ms / 1 kHz、IMU、四个电机插件、link
集合一致），唯一区别是 `homefly_maze.world` 把停机点移到较开阔的 `(2, 2)`。
原版机场 world 里没有独立的 `ground_plane` 模型：地面由 `model://runway` 自带的
无限 `<collision><plane>` 提供；lite 版则由 `grass` 模型提供。

**`GZ_SIM_RESOURCE_PATH`（只在用原版机场时需要）**：`model://runway` 由 gz 通过
该环境变量解析。`tools/run_wsl_sitl.sh` 已在脚本开头自动导出：

```bash
export GZ_SIM_RESOURCE_PATH="$ROOT/tools/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

手工起 GUI（8.7 / 8.8 的终端 1）时须自己在**同一个终端**同时导出两项，
缺一不可：

```bash
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$PWD/tools/gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r -v4 tools/gazebo/homefly_airport.world
```

`GZ_IP` 是 8.7 的 WSLg 组播发现必需项，机场场景同样要设；漏了会一直刷
`GUI requesting list of world names...`，GUI 连不上 server，`Entity Tree` 是空的。
漏了 `GZ_SIM_RESOURCE_PATH` 则 gz 报 `Unable to find uri[model://runway]`，或
试图在线下载该模型、server 卡在加载资源（日志提示 `server may be busy
downloading resources`），把机场模型丢掉。两个 `export` 都不会跨终端继承，
必须在起 `gz sim` 的那个终端里设；`GZ_IP` 也要给终端 2 的 bridge 设上。

GUI 起来的判据：`Entity Tree` 里能同时看到 `runway` 与 `homefly`，左下角仿真
时间在递增。此时若视口只有网格、看不到飞机，在 `Entity Tree` 里双击
`homefly`（或右键 Move to / Follow）把相机对到原点 `(0,0,0.2)`；仍只有网格
就加 `export LIBGL_ALWAYS_SOFTWARE=1` 后重启终端 1（WSLg 无 GPU 时 ogre2
可能起不来）。

**切换方式**：把 8.7 / 8.8 里终端 1 的 world 路径换掉，其余三个终端不变：

```bash
# GUI 迷宫 / 原版机场 / 轻量机场
gz sim -r -v4 tools/gazebo/homefly_maze.world
gz sim -r -v4 tools/gazebo/homefly_airport.world
gz sim -r -v4 tools/gazebo/homefly_airport_lite.world

# headless 回归（脚本已有 --world 参数，资源路径自动导出）
bash tools/run_wsl_sitl.sh --backend gz --world tools/gazebo/homefly_maze.world
bash tools/run_wsl_sitl.sh --backend gz --world tools/gazebo/homefly_airport.world
```

**关于「逐字节」**：`tools/gazebo/models/runway/` 是从
`ArduPilot/ardupilot_gazebo/models/runway` main 分支下载后未编辑的完整副本
（`model.sdf`、`model.config`、`meshes/airfield.dae` 与 `materials/textures/` 下
全部 7 张贴图），已逐个与上游 `raw.githubusercontent.com` 做 `cmp` 校验一致。
其中 SDF / DAE 实际引用的 5 张约 18 MB；另外 2 张
（`Grass004_2K_Color copy.jpg`、`Grass004_2K_Displacement.jpg`）上游存在但未被
引用，为保持目录逐字节也一并保留，不参与渲染。上游 `runway.sdf` 把跑道放在
`(-29, 545)` 供 GPS 导航用；这里 `<include>` 不带 `<pose>`，默认落在原点，
让飞机和跑道同处 GUI 视口 —— **唯一差异是 include 的摆放位置，模型资源本身
没有任何修改**。资源版权归 ArduPilot 项目。

**能力边界（重要）**：ArduPilot 文档里的迷宫是给带 360° 2D 激光雷达的 Iris 做
Cartographer SLAM / 自主导航用的；HomeFly 是**姿态自稳飞控**，只有
roll / pitch / yaw_rate / throttle，没有位置估计、避障或航点导航。场景能 100%
复用，但飞机不会自己穿迷宫 —— 只能在场景里悬停 / 手动打杆。另外迷宫墙是
**带碰撞体**的静态实体，打杆过猛撞墙会产生异常加速度并触发 `InvalidImuData`
Failsafe（粘性，可在 GCS 输 `arm` 或 `disarm` 恢复）；做纯净的姿态回归请用
默认的 `homefly_quad.world`。维护提示：四个文件里的 `homefly` 模型是同一份拷贝，
调整质量 / 惯量 / 电机常数时需要同步改四处；切换场景只需换 world 路径。
