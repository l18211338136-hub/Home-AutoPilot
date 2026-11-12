# 代码风格与命名

参考 PX4 的 C/C++ 风格，结合 HomeFly 统一约定。

## 命名（两套规则，不要混淆）

- **文件夹 / 文件名**：一律小写 `snake_case`，不使用 PascalCase。
- **代码标识符**：C++ 类名 PascalCase，成员 `_camel_case`，函数/变量 `snake_case`。

```
文件夹        flight_core/  control/  hal/  drivers/  platforms/linux/
C 源          drv_adc.c     drv_gpio.c
C 头          drv_adc.h     drv_gpio.h
C++ 源        flight_controller.cpp  attitude_estimator.cpp  pid_controller.cpp
C++ 头        flight_controller.hpp  attitude_estimator.hpp  pid_controller.hpp
```

C++ 源文件与头文件必须使用相同的基础文件名。驱动命名参考 PX4：`drv_dshot.h`、`drv_pwm_output.h`、`drv_board_led.h` 等。

日志输出统一使用 `HF_LOG_*` 宏，TAG 用大写模块名（`"IMU"`、`"MOTOR"`、`"BATTERY"`、`"FLIGHT"`）。禁止 `printf` / `cout` / `ESP_LOGx` / `HAL_UART_Transmit`。日志参数里不得出现有副作用的表达式（编译期级别剔除时不会求值）。

## 语言分工

| C11 | C++20 |
| --- | --- |
| MCU 厂商 SDK 适配、底层硬件驱动、与 C SDK 直接交互、内存/执行过程严格受限的底层模块 | FlightCore、控制算法、HAL 接口、传感器抽象、飞行状态管理、应用 DI、仿真与测试 |

## 格式

- LF 换行符。
- Tab 缩进，Tab 宽度 8。
- 行宽原则上不超过 140 字符。
- Allman 花括号（大括号独占一行），多参数虚函数分行。
- `public:` / `private:` / `protected:` 不额外缩进。
- 提交前执行格式检查与基础静态检查：

```bash
astyle --options=.astylerc --recursive "src/*.cpp,*.hpp" "simulation/*.cpp,*.hpp" "test/*.cpp,*.hpp"
clang-format --dry-run --Werror <changed files>
clang-tidy -p build/linux <changed files>
```

`.astylerc`、`.clang-format`、`.clang-tidy` 已放在仓库根目录。

## 提交前检查清单

- [ ] 新文件/目录名全部小写 snake_case。
- [ ] FlightCore 中没有出现任何 SDK 头文件或寄存器操作。
- [ ] 控制算法中没有 GPIO / SPI / I2C 访问。
- [ ] 新增硬件先走 HAL 抽象，驱动遵守统一接口。
- [ ] 新增功能有 Linux Native 测试。
- [ ] 涉及电机、飞控、电源的改动，写清失效安全行为与验证方式。
- [ ] 高频路径没有动态分配、阻塞、无界循环，也没有同步日志输出。
- [ ] 新增日志使用统一 `HF_LOG_*` 接口；高频 IMU/姿态/电机数据走 `FlightDataRecorder` + `FlightLogWriter`，落盘只在低优先级任务。
- [ ] 改动飞行日志二进制格式时同步更新 `kFlightLogVersion`、`docs/flight_log.md` 与 `tools/hfl_dump.py`。
- [ ] 改动 SITL 线协议时同步更新 `gazebo_protocol.hpp`、`tools/gazebo_bridge.py` 与 `test_gazebo_protocol.cpp` 的 golden 向量；控制循环内不得出现 socket 调用。
- [ ] `ctest` 全绿，且 `tools/hfl_dump.py` 对实际生成的 `.hfl` 校验为 `PASS`；SITL 改动另跑一次 `gazebo_bridge.py --self-test` 与 `--backend mock` 端到端。
