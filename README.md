# HomeFly

面向微型无人机的自主飞控系统研发项目。目标是构建一套**飞控软件与硬件解耦、平台无关、可移植、可扩展、可仿真**的飞控架构：更换 MCU、开发板或传感器，不应迫使整个飞控系统重写。

> 当前阶段：**Phase 1 — 架构验证**。Linux Native 可独立编译、仿真和测试；ESP32-S3 / STM32 为预留的板级与平台适配位，尚未接入真实硬件。

## 快速开始

### Linux / macOS（标准方式）

```bash
cmake -S . -B build/linux -G Ninja -DHOMEFLY_PLATFORM=linux
cmake --build build/linux
ctest --test-dir build/linux --output-on-failure
```

运行闭环仿真（默认 2000 个控制周期 = 1 kHz 下 2 秒），**每次运行都会生成二进制飞行日志** `homefly_flight.hfl`：

```bash
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000
./build/linux/src/applications/linux_native/homefly_linux_native --steps 1000 --failsafe
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000 --verbose --throttle 500
./build/linux/src/applications/linux_native/homefly_linux_native --flight-log logs/flight_001.hfl

# 离线校验完整性、导出 CSV 画图
python tools/hfl_dump.py homefly_flight.hfl --csv flight.csv --tail 10
```

### 接入 Gazebo SITL

飞控侧不链接任何 Gazebo 库；Gazebo 侧用独立桥接进程通过 UDP 帧对接。`--backend mock` 用纯 Python 复刻的刚体模型，**无需安装 Gazebo 即可端到端验证整条链路**：

```bash
# 终端 A
python tools/gazebo_bridge.py --backend mock --steps 2000
# 终端 B
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000 --gazebo
```

**一条命令跑完整流程**（自动构建、协议自测、桥接、飞控、`.hfl` 严格校验）：

```bash
bash tools/run_wsl_sitl.sh                    # mock，无需 Gazebo
bash tools/run_wsl_sitl.sh --backend gz       # WSL2 Ubuntu + 真 Gazebo（headless）
```

Windows 宿主接真实 Gazebo 请走 WSL2，并把 Gazebo / 桥接 / 飞控三者都放在 Linux 侧原生编译运行——WSL2 的 localhost 转发只支持 TCP，混合部署的 UDP 回环不通。完整 world 模板见 `tools/gazebo/homefly_quad.world`（片段为 `homefly_quad.sdf`），部署步骤、协议、轴系、时间契约与故障排查见 [docs/gazebo.md 第 8 节](docs/gazebo.md#8-wsl2--ubuntu-部署windows-宿主的推荐路径)。

输出统一走日志接口：

```
[00000000] [INFO ] [SYSTEM] HomeFly booting (platform=linux_native, loop=1000Hz, ...)
[00000001] [WARN ] [FLIGHT] mode disarmed -> stabilize (reason=0)
[00002000] [INFO ] [FLIGHTLOG] path=homefly_flight.hfl records=2000 blocks=125 dropped=0 write_failures=0 checksum=0xD590B8D5 footer=ok
[00002000] [INFO ] [LOGGING] emitted=15 drained=15 dropped(level=1 rate=0 full=0) sink_failures=0
```

### Windows（无 GNU 工具链时）

```bat
tools\build_windows_native.bat
```

该脚本加载 MSVC 环境后执行同一套 CMake + Ninja + CTest 流程。可通过 `VSINSTALL` 环境变量指定其它 VS 安装路径。

### 测试构建

```bash
cmake -S . -B build/test -G Ninja
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

GoogleTest 可用时自动使用；离线环境下退回 `test/homefly_test.hpp` 提供的 GTest 兼容最小测试框架，**CTest 始终可用，且不引入第三方依赖**。

## 已验证结果（本机 MSVC 19.51 / CMake 4.3 / Ninja）

| 项目 | 结果 |
| --- | --- |
| 编译目标 | 全量构建通过（C11 + C++20，MSVC `/W4` 无警告） |
| CTest | 13 / 13 通过，共 91 个用例（日志 13、飞行数据 6、飞行日志 10、Gazebo 协议 11、Gazebo 传输 7） |
| 编译期裁剪 | `HF_LOG_COMPILE_MIN_LEVEL=2` 独立构建 13 / 13 通过，DEBUG/INFO 宏整体消除 |
| 稳定场景 | 初始 roll 11.54° / pitch -6.87° → 2 s 后收敛到 ±0.3°，`PASS` |
| 失效保护场景 | IMU 掉线 → `Failsafe` + `ImuReadFault` + 电机归零，`PASS` |
| 飞行日志 | 2000 周期 → `144096` 字节 `.hfl`（2000 条 / 125 块 / `dropped=0`），离线 `--strict` 校验 `PASS`，CSV 导出 2000 行 |
| Gazebo SITL | mock 桥接 2000 帧锁步 `PASS`（`imu_rx=2000 motor_tx=2000 imu_drop=0 motor_drop=0`），收敛结果与内置 plant 一致；failsafe 场景同样双端 `PASS` |
| 文本日志 | 同周期只有 15 条，与飞行日志物理隔离 |

## 目录结构

```
HomeFly
├── src/
│   ├── hal/              HAL 接口（能力抽象，头文件库）
│   ├── control/          PID、互补滤波姿态解算、四轴 X 混控、数学类型
│   ├── configuration/    全部可调参数（PID 增益、周期、超时、电机范围）
│   ├── flight_core/      状态机、安全监控、FlightController、核心数据类型
│   ├── communication/    ILink 字节链路 + 遥测帧编码（不绑定 UART）
│   ├── drivers/          IPwmOutput → IMotor 适配、C11 板级原语
│   ├── platforms/
│   │   ├── common/logging/   统一日志 API、无锁环、结构化飞行数据、飞行日志写入器
│   │   ├── esp32/logging/    ESP32-S3 UART + SD 卡飞行日志后端（预留）
│   │   ├── stm32/logging/    STM32 UART + SD/Flash 后端（预留）
│   │   ├── linux/logging/   stdout + 真实 .hfl 文件落盘
│   │   └── linux/           LinuxClock / LinuxTimer
│   └── applications/
│       └── linux_native/ 组装根（依赖注入）、SimulationSource、主循环
├── simulation/           SimClock / SimPlant / SimImu / SimMotor / SimLink
│   └── gazebo/           Gazebo SITL 传输协议、UDP、HAL 适配器、SPSC 环
├── test/                 CTest + GoogleTest（或兼容最小框架）
├── boards/               esp32_s3 / stm32_f411 板级说明
├── cmake/                arm-none-eabi 交叉编译工具链模板
├── tools/                Windows 构建脚本、hfl_dump.py、gazebo_bridge.py、run_wsl_sitl.sh、Gazebo world/SDF
├── docs/                 架构、构建、日志、飞行日志、Gazebo SITL、实时约束、路线图、代码风格
├── CMakeLists.txt
└── SKILL.md              项目架构规范（本仓库的约定来源）
```

## 架构

```
             Applications (依赖注入 / 组装)
                   │
          ┌────────┴────────┐
          │                 │
      FlightCore          Drivers
          │                 │
          └────────┬────────┘
                   │
                  HAL
                   │
               Platforms
                   │
          MCU / SDK / RTOS
```

- **FlightCore 不依赖任何硬件**，只依赖 HAL 接口与配置数据结构。
- **控制算法只依赖抽象数据**，不访问 GPIO / SPI / I2C。
- **驱动实现 HAL 接口**；换传感器只换驱动，不改核心。
- **平台层适配 MCU / SDK / RTOS**；不同平台不得复制一份 FlightCore。
- `FlightController(IImu&, IMotor&, IClock&, const FlightConfig&)` 采用构造函数依赖注入。
- **仿真源可替换**：`SimulationSource` 是唯一注入点，内置 `SimPlant` 与 Gazebo SITL 共用同一套主循环、日志、遥测和飞行日志。
- **Logging 是公共基础设施**：所有模块统一使用 `HF_LOG_*`，平台差异只落在 `log_backend.cpp`，迁移平台不改业务模块。

详见 [docs/architecture.md](docs/architecture.md)。

## 日志基础设施

`src/platforms/common/logging/` 提供平台无关的统一 API（五个级别、毫秒时间戳、TAG、`HF_LOG_*` 宏）与**无阻塞前端**；`src/platforms/<platform>/logging/log_backend.cpp` 只负责实际输出。控制循环里的 `log_write()` 只做级别过滤、限流、有界格式化和无锁环形缓冲入队，**不做任何 I/O**；串口/文件/无线写入由低优先级的 `log_drain()` 驱动。高频 IMU、姿态、电机数据走独立的 `FlightDataRecorder` 二进制通道，不进文本日志。

完整规范见 [docs/logging.md](docs/logging.md)、[docs/gazebo.md](docs/gazebo.md)，并已写入 [SKILL.md](SKILL.md) 的基础设施规范。

## 飞行日志（Blackbox）

每个控制周期的 IMU / 姿态 / 电机输出以二进制 POD **真实落盘**到 `.hfl` 文件，与文本日志物理隔离。控制循环只做一次 POD 拷贝，低优先级任务攒满 16 条批量写一次文件，绝不逐帧 `fwrite`。

| 项 | 说明 |
| --- | --- |
| 文件结构 | 64 B 自描述 Header（magic / 版本 / 控制周期 / 调参指纹 / 平台）+ N×72 B 记录 + 32 B Footer |
| 完整性 | 全记录 FNV-1a 校验和、记录数、丢帧数、写失败数；Footer 缺失即判定崩溃 / 断电会话 |
| 缓冲 | 128 槽 SPSC 环（控制循环）+ 16 条块缓冲（低优先级任务），满即丢弃并计数，绝不等待 |
| 平台后端 | Linux 真实 `.hfl` 文件；ESP32-S3 / STM32 通过四个 BSP C 钩子接 SD / Flash（预留） |
| 离线工具 | `tools/hfl_dump.py`：完整性校验、摘要、`--csv` 导出、`--tail` 查看末段 |

```bash
python tools/hfl_dump.py homefly_flight.hfl --csv flight.csv --tail 10
# verification : PASS
```

完整格式、实时性论证与真机移植清单见 [docs/flight_log.md](docs/flight_log.md)。

## 关键控制路径约束

| 项 | 约定 |
| --- | --- |
| 执行频率 | 1 kHz（`control_period_us = 1000`） |
| IMU 数据更新时间 | 采样时间戳距当前 ≤ 5 ms |
| 超时策略 | 超过 3 个周期 → `ControlOverrun` |
| 故障处理 | 进入粘性 `Failsafe`，记录原因，立即断臂并输出最小值 |
| 安全状态 | `Disarmed` / `Failsafe`，电机输出为 `motor_output_min` |
| 禁止项 | `step()` 内动态分配、阻塞、文件/网络 IO、同步日志输出、无界循环 |
| 日志 / 飞行日志 | 控制循环只入队（零 I/O）；缓冲满即丢弃并计数；`log_drain()` 与 `FlightLogWriter::pump()` 只在低优先级任务调用 |
| Gazebo SITL | UDP 收发只在 `GazeboTransport` 后台任务；控制循环只读写 SPSC 环；`GazeboClock` 由 Gazebo 仿真时间驱动，不用宿主墙钟 |

详见 [docs/realtime_contract.md](docs/realtime_contract.md) 与 [docs/logging.md](docs/logging.md)。

## 路线图

遵循「先飞起来，再逐步完善」，每个阶段都必须有可验证交付。详见 [docs/roadmap.md](docs/roadmap.md)。

1. ✅ Phase 1：Linux Native 架构闭环 + 统一日志基础设施 + 可落盘飞行日志 + Gazebo SITL 传输（本版本）
2. ⬜ Phase 2：真实 Gazebo 联调（gz 后端）+ ESP32-S3 平台适配（UART/USB CDC + SD 卡飞行日志后端） + 真实 IMU/电机驱动 + 台架测试
3. ⬜ Phase 3：首次稳定悬停
4. ⬜ Phase 4：STM32 系列适配、自研 PCB
5. ⬜ Phase 5：飞行模式、返航、定位、多传感器融合

最终原则：**软件定义飞控，硬件实现能力**。
