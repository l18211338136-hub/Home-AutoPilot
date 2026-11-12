HomeFly 是一个面向微型无人机的自主飞控系统研发项目。

核心目标是构建一套飞控软件与硬件解耦、平台无关、可移植、可扩展、可仿真的飞控架构。

系统不应绑定某一种 MCU、开发板、传感器或 RTOS。只要硬件平台实现了约定的接口，就能够接入 HomeFly Flight Architecture。
1. FlightCore 不依赖具体硬件。
2. HAL 统一抽象硬件能力。
3. Drivers 负责具体传感器和外设驱动。
4. Platforms 负责 MCU、SDK 和 RTOS 适配。
5. Applications 负责系统组装和启动。
6. Linux Native 支持独立编译、测试和仿真。
7. 同一套核心代码能够运行在不同 MCU 平台。
8. 采用接口优先、依赖注入的设计方式。
C11：
- MCU 厂商 SDK 适配。
- 底层硬件驱动。
- 需要与 C SDK 直接交互的代码。
- 对内存和执行过程要求严格控制的底层模块。
C++20：
- FlightCore。
- 控制算法。
- HAL 接口。
- 传感器抽象。
- 飞行状态管理。
- 日志基础设施、结构化飞行数据记录与飞行日志（黑匣子）。
- 应用层依赖注入。
- 仿真和测试框架。
飞控实时路径应避免不可控的动态内存分配、阻塞操作和不确定执行时间。
采用分层架构，将飞控核心与硬件平台分离。
```
HomeFly
│
├── src/
│   ├── flight_core/
│   ├── control/
│   ├── hal/
│   ├── drivers/
│   ├── platforms/
│   ├── communication/
│   ├── configuration/
│   └── applications/
│
├── boards/
│   ├── esp32_s3/
│   └── stm32_f411/
│
├── simulation/
│
├── test/
│
├── tools/
│
├── docs/
│
├── CMakeLists.txt
└── README.md
```
日志与飞行日志基础设施（所有平台共用同一套接口）：
```
src/platforms/
├── common/logging/
│   ├── log_level.hpp / log_sink.hpp / log_backend.hpp
│   ├── log.hpp / log.cpp                    统一文本日志 API、无锁 MPMC 环、过滤/限流/统计
│   ├── flight_data.hpp / flight_data.cpp   结构化飞行数据 POD + 128 槽 SPSC 环
│   ├── flight_log.hpp / flight_log.cpp     飞行日志写入器（.hfl 格式、16 条批量块、校验和、Footer）
│   └── flight_log_storage.hpp              平台存储工厂声明（链接期绑定）
│
├── esp32/logging/   log_backend.cpp + flight_log_sink.cpp
├── stm32/logging/   log_backend.cpp + flight_log_sink.cpp
└── linux/logging/   log_backend.cpp + file_log_sink.cpp + flight_log_sink.cpp

tools/
├── hfl_dump.py                            离线校验 / CSV 导出 / 截断检测
├── gazebo_bridge.py                       mock / gz / classic 三后端 SITL 桥接进程
├── run_wsl_sitl.sh                        WSL2 一键 SITL（构建 + 桥接 + Gazebo + .hfl 校验）
└── gazebo/
    ├── homefly_quad.sdf                   Gazebo Garden Quad-X 模型片段
    └── homefly_quad.world                 完整 world（1 kHz 物理、rotor joints、IMU、4 电机插件）
```
外部仿真器（SITL）基础设施，飞控侧不依赖任何仿真器 SDK：
```
simulation/gazebo/
├── gazebo_protocol.hpp/.cpp    64 字节自描述小端 UDP 帧、FNV-1a 校验和、seq 去重
├── spsc_ring.hpp               无锁 SPSC 环（控制循环 <-> 后台任务）
├── udp_socket.hpp/.cpp         Winsock / POSIX 条件编译的 non-blocking UDP
├── gazebo_transport.hpp/.cpp   唯一承载 socket I/O 的低优先级任务 + 丢弃/失败统计
└── gazebo_hal.hpp/.cpp         GazeboImu:IImu / GazeboMotor:IMotor / GazeboClock:IClock

src/applications/linux_native/
└── simulation_source.hpp/.cpp  SimulationSource：内置 plant 与外部 SITL 的唯一注入点
```
分两部分：
- simulation/gazebo：实现 IImu / IMotor / IClock 的传输适配器，所有飞控模块不改。
- tools/gazebo_bridge.py：独立进程，把 Gazebo IMU 话题翻译成 HomeFly 帧、把电机帧翻译成 Actuators；mock 后端用纯 Python 复刻刚体模型，无需安装 Gazebo 即可端到端验证。

核心约束：飞控不链接 Gazebo / protobuf / ZeroMQ；控制循环只读写 SPSC 环，socket I/O 只发生在低优先级任务；每个控制周期恰好一帧 IMU / 一帧电机，仿真时间步长必须等于 control_period_us；GazeboClock 由「被消费的 IMU 帧时间戳」推进，禁止用宿主墙钟。线协议变更必须同步更新桥接脚本与 golden 向量测试。

部署约束：外部仿真器、桥接进程与飞控必须处于同一 UDP 网络命名空间。WSL2 的 `localhostForwarding` 只转发 TCP，Windows 的 `.exe` 与 WSL2 桥接走 `127.0.0.1` UDP 必然失败；在 WSL2 中必须把飞控重新编译成 Linux 原生 ELF，三者全部放 Linux 侧（或全部放 Windows 侧）。一键入口 `tools/run_wsl_sitl.sh`，完整 world 模板 `tools/gazebo/homefly_quad.world`，细节见 `docs/gazebo.md` 第 8 节。

WiFi 地面站控制链路（上行指令，与 SITL 同一套架构模式）：
```
src/communication/
├── control_protocol.hpp/.cpp   28 字节自描述小端帧（magic "HFLC"/version/type/seq/payload[16]/FNV-1a 校验）
│                               类型：Arm=1 Disarm=2 Setpoint=3 Ping=4 Mode=5；Setpoint payload = 4 个 float
└── command_receiver.hpp/.cpp   poll() 唯一做 I/O（低优先级任务）；latest_setpoint() 控制循环调用、零 I/O
                                ARM/DISARM 锁存；链路超时强制断臂（link_timeout_us=0 禁用）

src/applications/linux_native/
└── udp_link.hpp/.cpp           ILink 的 non-blocking UDP 实现（复用 simulation/gazebo/udp_socket.hpp）
```
tools/wifi_gcs.py：极简 Python 地面站。ARM → 20Hz SETPOINT → Ctrl-C 发 DISARM；`--rearm-every N` 周期性 DISARM+ARM（间隔 150ms 分两帧发送）——FSM 粘性 Failsafe 只能靠 DisarmRequest 离开，活链路下这是唯一恢复路径，且两帧背靠背会被单次 poll() 合并消费而互相抵消。

核心约束：FlightCore 零改动；控制循环只读 latest_setpoint()（latched 状态 + 时钟，无 socket I/O）；所有 UDP 收发只在低优先级 logging 任务里 poll()；链路超时降级为断臂（复用 SafetyMonitor 模式）。验证：
- 确定性集成测试 test/communication/test_wifi_integration.cpp（真实 UdpLink 环回 → CommandReceiver → FlightController，虚拟时钟，含断臂测试）——CTest 必绿。
- 真实二进制 e2e：`homefly_linux_native --realtime --steps 13000 --wifi --link-timeout 500` + `python tools/wifi_gcs.py --rate 20 --count 300 --rearm-every 20` → `result: PASS`。

宿主实时性已知问题（Windows）：默认调度器 tick ~15.6ms 会让 sleep_for(1ms) 周期性过冲 3ms overrun 阈值。已修：LinuxTimer 构造时 timeBeginPeriod(1)（winmm）+ 尾部 400us 忙等自旋；--realtime 时控制线程 THREAD_PRIORITY_HIGHEST、日志线程 BELOW_NORMAL（main.cpp）。Linux/真 MCU 不受影响。

WSL2 已知问题：宿主调度抖动仍会偶发超过 3ms overrun 阈值（一次 ControlOverrun -> 粘性 Failsafe，re-arm 恢复在持续抖动下不可靠）。缓解：FC 加 `--overrun-factor N` 放宽阈值（N x 1ms，建议 WSL2 用 8~10）；e2e 命令示例 `homefly_linux_native --realtime --steps 13000 --wifi --link-timeout 500 --overrun-factor 10`。


飞控系统核心。

负责：
- 飞行状态管理。
- 飞行模式管理。
- 飞行任务调度。
- 飞行安全逻辑。
- 核心数据结构。
- 飞行状态机。
不得直接依赖 ESP-IDF、STM32 HAL、Arduino 或具体传感器驱动。
控制算法层。

负责：
- PID 控制器。
- 姿态解算。
- 角速度控制。
- 姿态控制。
- 高度控制。
- 电机混控。
- 控制分配。
控制算法只依赖抽象数据和数学库，不直接操作硬件。
硬件抽象层。

定义统一硬件接口，例如：
```
class IImu
{
public:
    virtual ~IImu() = default;

    virtual bool initialize() = 0;

    virtual bool read(
        ImuData& data
    ) = 0;
};
```
其他接口包括：
- IBarometer
- IMagnetometer
- IGyroscope
- IAccelerometer
- IMotor
- IPwmOutput
- ISpi
- II2c
- IUart
- IClock
- ITimer
- IStorage
接口应表达硬件能力，不暴露具体芯片寄存器、SDK 类型或平台实现细节。
具体硬件驱动实现。

例如：
```
drivers/
├── imu/
│   ├── drv_mpu6050.cpp
│   ├── drv_mpu6050.hpp
│   ├── drv_icm42688.cpp
│   └── drv_icm42688.hpp
│
├── motor/
│   ├── drv_dshot.cpp
│   ├── drv_dshot.hpp
│   ├── drv_pwm.cpp
│   └── drv_pwm.hpp
│
└── sensors/
    ├── drv_bmp280.cpp
    └── drv_bmp280.hpp
```
驱动负责将具体硬件转换成 HAL 定义的统一接口。

更换传感器时，原则上只需要更换驱动实现，不应修改 FlightCore 和控制算法。
硬件平台适配层。
```
platforms/
├── esp32/
│   ├── esp32_clock.cpp
│   ├── esp32_clock.hpp
│   ├── esp32_gpio.cpp
│   ├── esp32_gpio.hpp
│   └── esp32_timer.cpp
│
├── stm32/
│   ├── stm32_clock.cpp
│   ├── stm32_gpio.cpp
│   └── stm32_timer.cpp
│
└── linux/
    ├── linux_clock.cpp
    ├── linux_timer.cpp
    └── linux_uart.cpp
```
平台层负责：
- MCU 初始化。
- 时钟配置。
- GPIO。
- SPI / I2C / UART。
- PWM / 定时器。
- 中断。
- DMA。
- RTOS 适配。
- 平台相关系统服务。
Linux Native 平台用于仿真、测试和开发，不代表 MCU 飞控固件运行在 Linux 上。
Linux Native 必须同时支持两种仿真源：内置确定性 SimPlant（默认、可复现）与外部 Gazebo SITL（--gazebo，由 Gazebo 仿真时间驱动）。两者共用同一主循环、同一日志与飞行日志，切换不得改动任何业务模块。
通信协议层。

负责：
- 遥控器通信。
- 地面站通信。
- MAVLink 等协议适配。
- 遥测数据。
- 命令接收。
- 飞行状态上报。
通信协议不应与具体 UART 或无线芯片绑定。
日志基础设施层。

与通信、传感器、执行器并列，属于所有模块都可以使用的公共接口。从第一阶段 ESP32-S3 起就必须建立统一日志接口，后续迁移到 STM32、Linux 或 PX4 风格架构时，业务模块不得因平台更换而修改。

分三部分：
- platforms/common/logging：平台无关的统一 API（文本日志前端 + 飞行日志写入器），所有飞控模块调用。
- platforms/<platform>/logging/log_backend.cpp：不同硬件平台的 Console 输出实现。
- platforms/<platform>/logging/flight_log_sink.cpp：飞行日志存储（文件 / SD / Flash），通过 flight_log_storage_* 链接期绑定。

统一日志级别：
- DEBUG：调试信息。
- INFO：正常运行信息。
- WARN：警告信息。
- ERROR：错误信息。
- FATAL：致命错误，任何构建配置下都不得被过滤。

统一 API 使用 HF_LOG_DEBUG / HF_LOG_INFO / HF_LOG_WARN / HF_LOG_ERROR / HF_LOG_FATAL 宏，调用 log_write(LogLevel, tag, format, ...)。业务模块禁止直接使用 printf、cout、ESP_LOGx、HAL_UART_Transmit 等平台相关输出。

串口输出格式：
```
[00001234] [INFO ] [SYSTEM] HomeFly booting
[00001300] [WARN ] [BATTERY] Low voltage: 3.45V
[00001320] [ERROR] [IMU] Sensor read failed
```
- 时间戳：系统启动后的毫秒数，8 位零填充。
- 级别：5 字符定宽标签。
- TAG：所属模块。
- MESSAGE：有界格式化内容，整行有硬上限。

输出通道通过 ILogSink 抽象扩展，业务代码不感知：
```
飞控业务模块 IMU / MOTOR / BATTERY / FLIGHT / NAVIGATION
        ↓
Common Logging API：级别过滤、时间戳、格式化、限流
        ↓
Console（USB CDC / UART）| File（SD / Flash）| Telemetry（无线传输）
```
第一阶段 ESP32-S3 先实现 USB CDC / UART 输出；飞行日志在 Linux Native 已真实落盘（ESP32-S3 / STM32 为 BSP 钩子占位），后续增加 SD 卡与无线遥测时业务模块不改。

实时性硬要求：日志不得阻塞飞控控制循环。
1. 使用静态环形缓冲区，禁止动态内存分配。
2. 前端只入队，日志异步输出任务（低优先级 / 空闲任务）负责实际 I/O。
3. 提供编译期与运行期双层日志等级过滤。
4. 提供高频日志限流。
5. 统计串口发送失败与缓冲区丢弃数量。
6. 实时飞行数据与普通调试日志必须分离。
7. IMU、姿态角、电机输出等高频数据禁止使用普通文本日志，必须走独立的结构化飞行数据记录机制（二进制 POD + 独立环形缓冲），并落盘为飞行日志。
8. 环形缓冲满时丢弃并计数，绝不等待；控制循环内禁止调用 drain 或任何同步输出。

飞行日志（Blackbox）是**独立的可落盘二进制记录**，第一阶段就必须有可验证交付（真实文件 + 离线解析工具），不得只留内存占位。

- 文件格式：64 字节自描述 Header（magic / version / record_size / session_id / control_period_us / config_hash / platform）+ N 条固定长度记录 + 32 字节 Footer（记录数 / 丢帧数 / 写失败数 / 全记录校验和）。Footer 缺失即判定崩溃或断电会话。
- 两级缓冲：前端 SPSC 环（控制循环只做一次 POD 拷贝）+ 后端 16 条块缓冲（攒满才写一次），禁止逐帧 fwrite / f_write。
- 实际落盘只允许发生在低优先级日志任务；控制循环内禁止 pump / finish / 任何文件 IO。
- 缓冲满即丢弃并计数，写入失败保留记录供下次重试，两者都计入 Footer，不得静默失败。
- 平台存储通过 flight_log_storage_open / flight_log_storage / flight_log_storage_close 链接期绑定；common 层不得包含任何 SDK 或文件系统头文件；MCU 由 BSP 提供 mount / write / flush / close 四个 C 钩子。
- 必须提供离线解析工具：校验 magic、版本、尺寸、记录数与校验和，检测截断会话，导出 CSV；校验失败返回非零退出码，可接入 CI。
- 修改二进制格式必须同步提升 version、更新格式文档与解析工具，保证旧日志仍可识别。

配置管理层。

负责：
- 飞控参数。
- PID 参数。
- 传感器配置。
- 电机布局。
- 飞行器类型。
- 板级配置。
硬件差异通过配置和平台适配解决，不应通过大量修改核心代码解决。
系统组装层。

负责：
- 依赖注入。
- 硬件实例化。
- 模块注册。
- 系统初始化。
- 任务创建。
- 飞控主循环。
不同平台可以拥有不同的启动入口，但复用相同的核心模块。
必须遵循依赖倒置原则。
```
             Applications
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
- FlightCore 不允许直接调用 ESP-IDF API。
- FlightCore 不允许直接调用 STM32 HAL API。
- 控制算法不允许直接访问 GPIO、SPI、I2C。
- 驱动实现必须遵守 HAL 接口。
- 平台适配必须遵守统一的平台接口。
- 不同硬件平台不得复制一份 FlightCore。
优先使用构造函数依赖注入：
```
class FlightController
{
public:
    explicit FlightController(
        IImu& imu,
        IMotor& motor
    )
        : _imu(imu),
          _motor(motor)
    {
    }

private:
    IImu& _imu;
    IMotor& _motor;
};
```
高频实时路径可以根据性能需要使用模板、静态多态或其他零开销抽象方式。

不应为了统一形式而牺牲实时性能。
当前主要开发平台。

目标：
- 验证飞控核心架构。
- 验证 IMU 数据采集。
- 验证姿态解算。
- 验证 PID 控制。
- 验证电机输出。
- 实现微型无人机首次稳定飞行。
使用 ESP-IDF 官方工具链和构建体系。
支持：
- STM32F411。
- STM32F405。
- STM32F7。
- STM32H7。
不同 STM32 芯片通过平台适配实现兼容。
根据 HomeFly 飞行需求设计自有飞控硬件。

自研 PCB 应优先复用现有 HAL、Drivers 和 FlightCore。
逐步扩展：
- 飞行模式。
- 自动返航。
- 室内定位。
- 视觉辅助。
- 机库对接。
- 自动换电。
- 多传感器融合。
统一使用 CMake + Ninja 管理项目。

但不强行替换各 MCU 厂商的官方构建体系。
用于核心算法开发、仿真和单元测试。
```
cmake -S . -B build/linux \
    -G Ninja \
    -DHOMEFLY_PLATFORM=linux

cmake --build build/linux
```
使用 ESP-IDF 官方构建系统：
```
idf.py set-target esp32s3

idf.py build

idf.py flash monitor
```
使用 ARM GNU Toolchain 和 CMake Toolchain：
```
cmake -S . -B build/stm32 \
    -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
    -DHOMEFLY_PLATFORM=stm32

cmake --build build/stm32
```
- 公共模块通过 CMake 管理。
- 日志前端 homefly_logging_common 是平台无关静态库，每个平台单独提供 homefly_log_backend（含 Console sink 与 flight_log_storage_* 工厂），链接期绑定，不产生运行时分发开销。
- 通过 HF_LOG_COMPILE_MIN_LEVEL 在编译期剔除低等级日志，生产固件零开销。
- 飞行日志必须真实落盘，并由离线工具校验；格式版本变更要同步更新解析工具。
- 平台模块独立编译。
- Linux Native 可以独立构建 FlightCore。
- ESP32 使用 ESP-IDF 生成固件。
- STM32 使用交叉编译生成固件。
- 不允许为了统一构建而破坏官方 SDK 的正常工作方式。
HomeFly 统一采用 PX4 风格的小写 snake_case 文件夹和文件命名。

文件夹和文件名不使用 PascalCase。
全部使用小写 snake_case：
```
flight_core/
control/
hal/
drivers/
platforms/
communication/
configuration/
applications/
```
C 源文件：
```
drv_adc.c
drv_gpio.c
drv_uart.c
```
C 头文件：
```
drv_adc.h
drv_gpio.h
drv_uart.h
```
C++ 源文件：
```
flight_controller.cpp
attitude_estimator.cpp
pid_controller.cpp
```
C++ 头文件：
```
flight_controller.hpp
attitude_estimator.hpp
pid_controller.hpp
```
C++ 源文件与头文件使用相同的基础文件名。
参考 PX4 的驱动命名方式：
```
drv_adc.h
drv_bitbang_uart.h
drv_board_led.h
drv_dshot.h
drv_input_capture.h
drv_neopixel.h
drv_pwm_output.h
drv_sensor.h
drv_watchdog.h
```
HomeFly 可根据具体驱动类型扩展命名。
注意：文件命名与代码标识符命名是两套规则。不能因为 C++ 类名使用 PascalCase，就将 C++ 文件名也改为 PascalCase。
参考 PX4 的 C/C++ 代码风格，并结合 HomeFly 项目统一约定。
- 使用 LF 换行符。
- 使用 Tab 缩进，Tab 宽度为 8。
- 行宽原则上不超过 140 字符。
- 使用统一的花括号和换行格式。
- public:、private:、protected: 不额外缩进。
- 使用 AStyle 检查和格式化代码。
- 使用 clang-tidy 进行静态检查。
代码提交前应通过格式检查和基础静态检查。
飞控属于实时控制系统，必须优先考虑确定性和安全性。

禁止或谨慎使用：
- 高频控制路径中的无约束动态内存分配。
- 不可预测的阻塞操作。
- 控制循环中的同步日志输出（printf / 串口写入）、日志 drain 与飞行日志 pump / finish。
- 高频路径中的文件或网络 IO，尤其逐帧写飞行日志。
- 无界循环。
- 不可控的锁竞争。
- 在中断中执行复杂逻辑。
- 未经验证的传感器数据。
- 未经保护的电机输出。
关键控制路径必须明确：
- 执行频率。
- 最大执行时间。
- 数据更新时间。
- 超时策略。
- 故障处理策略。
- 安全状态。
任何传感器或电机异常都应有明确的降级或保护行为。
日志同样属于实时路径的一部分：前端 log_write 必须是无分配、无阻塞、无 I/O、有界执行时间的；实际输出只能由低优先级异步任务完成。飞行日志前端只允许一次 POD 拷贝，批量落盘同样由该任务完成。日志缓冲满、限流、级别过滤、输出失败、飞行日志丢帧都必须有计数，不得静默失败。
所有不依赖具体硬件的核心模块都应支持 Linux Native 测试。
外部仿真器接入属于可替换的仿真源，不得因此修改 FlightCore、日志、遥测或飞行日志；Linux Native 主循环对内置 plant 与外部 SITL 必须完全统一，唯一分支在应用组装层的 SimulationSource。

测试体系：
```
test/
├── flight_core/
├── control/
├── hal/
├── drivers/
├── platforms/
└── simulation/
```
使用 GoogleTest + CTest。
```
cmake -S . -B build/test -G Ninja

cmake --build build/test

ctest --test-dir build/test --output-on-failure
```
测试重点：
- PID 算法。
- 姿态解算。
- 坐标变换。
- 电机混控。
- 飞行状态机。
- 传感器数据异常。
- 控制超时。
- 故障保护。
- 日志级别过滤、输出格式、限流、缓冲满丢弃、输出失败统计、多生产者并发。
- 结构化飞行数据记录的顺序、溢出与失败重试。
- 飞行日志 Header/Footer 自描述字段、批量块、校验和、写入失败重试与 Footer 统计；实际生成的日志文件必须由离线工具读回校验。
- SITL 线协议（golden 向量跨语言一致、校验和、magic/version/type、非有限值）、UDP 回环、seq 去重、等待超时、SPSC 环溢出与 HAL 互锁。
- 外部仿真场景必须与内置 plant 场景做相同的收敛 / 失效保护判定，并生成可由离线工具校验的飞行日志。
尽量使用模拟硬件接口进行核心算法测试。
AI 开发助手必须遵循以下原则：
1. 优先理解现有目录结构和架构。
2. 不得擅自修改核心架构。
3. 不得将平台相关代码放入 FlightCore。
4. 不得让 FlightCore 直接依赖具体 MCU SDK。
5. 新增硬件必须优先考虑 HAL 抽象。
6. 新增驱动必须遵守统一接口。
7. 新增平台必须尽可能复用已有核心代码。
8. 新增功能必须考虑 Linux Native 测试。
9. 不得随意增加第三方依赖。
10. 不得为了快速实现功能而破坏硬件与软件解耦。
11. 所有新增文件夹和文件名必须使用小写 snake_case。
12. 所有 C++ 文件名必须使用小写 snake_case，不得使用 PascalCase 文件名。
13. 修改文件前先分析依赖关系，避免跨层直接调用。
14. 涉及电机、飞行控制和电源的修改，必须考虑失效安全和测试方式。
15. 所有模块输出日志必须使用统一 HF_LOG_* 接口，不得使用平台相关输出函数；新增平台只需提供 log_backend.cpp 输出实现。
16. 新增日志调用必须评估实时性：控制循环内只允许前端入队，高频数据必须走结构化飞行数据记录，不得引入同步 I/O 或动态分配。
17. 飞行日志（黑匣子）必须与文本日志分离、真实落盘、可离线校验；控制循环内只做一次 POD 拷贝，落盘与 Footer 写入只由低优先级任务完成，禁止逐帧写文件；新增平台只需实现 flight_log_storage_* 工厂与 BSP 钩子，不改变 .hfl 格式。
18. 接入外部仿真器（Gazebo / PX4 SITL / HIL）只能通过替换 SimulationSource 与实现 IImu / IMotor / IClock 适配器完成；飞控核心不得依赖仿真器 SDK，控制循环不得出现 socket 或任何同步 I/O，外部时间源必须单调递增且与 control_period_us 同频；线协议必须自描述、带校验和、可去重并可离线复现。
研发遵循先飞起来，再逐步完善自研飞控系统的原则。
每个阶段必须有可验证的交付结果，不允许在基础飞行能力尚未验证时过早扩展复杂功能。
HomeFly 最终应形成以下能力：
- 一套独立的 FlightCore。
- 一套统一的 HAL。
- 一套可替换的 Drivers。
- 一套平台无关的统一日志基础设施、结构化飞行数据记录与可落盘飞行日志（Blackbox）。
- 一套可替换的外部仿真器（SITL/HIL）传输层，业务模块零改动。
- 多种 MCU 平台适配。
- 独立的 Linux Native 仿真和测试环境。
- 可扩展的控制算法体系。
- 可逐步升级的自研飞控硬件。
最终原则：软件定义飞控，硬件实现能力；更换 MCU、开发板或传感器，不应迫使整个飞控系统重写。



