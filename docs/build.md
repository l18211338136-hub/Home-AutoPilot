# 构建说明

统一使用 **CMake + Ninja** 管理公共模块，但不强行替换 MCU 厂商的官方构建体系。

## Linux Native（核心算法 / 仿真 / 单元测试）

```bash
cmake -S . -B build/linux -G Ninja -DHOMEFLY_PLATFORM=linux
cmake --build build/linux
ctest --test-dir build/linux --output-on-failure
```

仅构建+测试（不产 MCU 固件）：

```bash
cmake -S . -B build/test -G Ninja
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

运行仿真：

```bash
./build/linux/src/applications/linux_native/homefly_linux_native --help
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000
./build/linux/src/applications/linux_native/homefly_linux_native --steps 1000 --failsafe
./build/linux/src/applications/linux_native/homefly_linux_native --steps 1000 --realtime
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000 --verbose --throttle 500
./build/linux/src/applications/linux_native/homefly_linux_native --flight-log logs/flight_001.hfl
./build/linux/src/applications/linux_native/homefly_linux_native --no-flight-log

# Gazebo SITL（另一个终端先启动桥接）
python tools/gazebo_bridge.py --backend mock --steps 2000
./build/linux/src/applications/linux_native/homefly_linux_native --steps 2000 --gazebo
```

`--realtime` 用宿主单调时钟按 1 kHz pace 主循环；默认使用 `SimClock` 的确定性虚拟时间，便于复现。`--gazebo` 改由 Gazebo 的 IMU 帧驱动节拍（此时 `--realtime` 被忽略），完整协议与端口参数见 [gazebo.md](gazebo.md)。

> 注意：Windows / 通用 OS 不是实时系统，1 ms 定时抖动可能超过 3 个周期而触发 `ControlOverrun` 粘性 Failsafe。该模式用于观察真实时钟节拍下的日志与落盘行为（例如验证飞行日志 `dropped=0`），**不要用它的 `result` 判定控制收敛**；收敛验证请使用默认的虚拟时钟模式。默认每次运行生成 `homefly_flight.hfl`（已被 `.gitignore` 忽略）：

```bash
python tools/hfl_dump.py homefly_flight.hfl                     # 摘要 + 完整性校验
python tools/hfl_dump.py homefly_flight.hfl --csv flight.csv   # 导出 CSV
```

解析工具为纯标准库 Python 3.10+，校验失败返回非零退出码，可直接接入 CI。

## Windows 宿主

```bat
tools\build_windows_native.bat
```

脚本会：

1. 调用 `vcvars64.bat` 初始化 MSVC；
2. 显式补齐 `INCLUDE` / `LIB` / Windows SDK `bin` 路径（应对注册表查询受限的环境）；
3. 执行 `cmake -G Ninja` + `cmake --build` + `ctest`。

若 VS 不在默认路径，先设置 `set VSINSTALL=<你的 VS 安装目录>`。

## WSL2 Ubuntu（接真实 Gazebo）

同一个 `cmake -S . -B build/wsl -G Ninja -DHOMEFLY_PLATFORM=linux` 在 WSL2 里直接可用（非 MSVC 分支为 `-Wall -Wextra -Wpedantic`，`udp_socket.cpp` 走 POSIX socket，线程库由 `Threads::Threads` 显式链接）。源码建议放在 WSL2 的 ext4 而不是 `/mnt/d`。

```bash
cd ~/Home-Fly
cmake -S . -B build/wsl -G Ninja -DHOMEFLY_PLATFORM=linux
cmake --build build/wsl
ctest --test-dir build/wsl --output-on-failure
bash tools/run_wsl_sitl.sh              # 自动构建 + mock 锁步 + .hfl 校验
bash tools/run_wsl_sitl.sh --backend gz # 真 Gazebo
```

注意 **Windows 的 `.exe` 不能与 WSL2 的桥接直接走 UDP 回环**（WSL2 的 localhost 转发仅支持 TCP），飞控必须在 WSL2 里重新编译成原生 ELF。完整部署与排障见 [gazebo.md 第 8 节](gazebo.md#8-wsl2--ubuntu-部署windows-宿主的推荐路径)。

## ESP32-S3（预留，使用 ESP-IDF 官方构建体系）

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

公共模块通过 ESP-IDF 组件引用 `src/` 下的静态库源码，**不把 ESP-IDF API 写进 FlightCore**。详见 `boards/esp32_s3/README.md`。

## STM32（预留，交叉编译）

```bash
cmake -S . -B build/stm32 -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
    -DHOMEFLY_PLATFORM=stm32

cmake --build build/stm32
```

`cmake/arm-none-eabi.cmake` 是模板，需按实际芯片调整 `-mcpu` / `-mfpu` 与链接脚本。

## CMake 目标一览

| 目标 | 类型 | 说明 |
| --- | --- | --- |
| `homefly_include` | INTERFACE | 统一 include 根（`src/` 与项目根） |
| `homefly_hal` | INTERFACE | HAL 纯接口 |
| `homefly_control` | STATIC | PID / 姿态解算 / 混控 |
| `homefly_configuration` | STATIC | 默认参数 |
| `homefly_flight_core` | STATIC | 状态机 / 安全 / FlightController |
| `homefly_communication` | STATIC | 遥测编码 |
| `homefly_drivers` | STATIC | PWM 电机驱动 + C11 板级原语 |
| `homefly_platform_linux` | STATIC | Linux 时钟 / 定时器 |
| `homefly_simulation` | STATIC | 仿真 HAL 实现 + Gazebo SITL 传输/协议/UDP 适配器（Windows 链接 `ws2_32`） |
| `homefly_logging_common` | STATIC | 平台无关日志前端 + 结构化飞行数据 + 飞行日志写入器 |
| `homefly_log_backend` | STATIC | 当前平台的 Console 输出 + 飞行日志存储工厂（每平台一份） |
| `homefly_linux_native` | EXECUTABLE | 仿真入口（`main.cpp` + `simulation_source.cpp`，含低优先级日志任务） |
| `test_*` | EXECUTABLE | 13 个 CTest 目标 / 91 个用例 |

### 编译期日志级别过滤

```bash
cmake -S . -B build/release -G Ninja \
    -DHOMEFLY_PLATFORM=linux -DHF_LOG_COMPILE_MIN_LEVEL=2
```

`0=DEBUG 起、1=INFO 起、2=WARN 起、3=ERROR 起、4=仅 FATAL`。被剔除的 `HF_LOG_*` 宏展开为空，生产固件零开销；`FATAL` 永不过滤。运行期仍可用 `log_set_level()` 调整。

## 测试框架说明

`test/CMakeLists.txt` 使用 `find_package(GTest QUIET)`：

- 找到 GoogleTest → 链接 `GTest::gtest_main`；
- 未找到 → 使用 `test/homefly_test.hpp` 内的兼容最小实现（同样的 `TEST` / `EXPECT_*` 写法）。

这样既符合「GoogleTest + CTest」的约定，也满足「不随意增加第三方依赖」的约束。
