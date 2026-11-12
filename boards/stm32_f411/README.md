# STM32F411

**状态：预留适配位，尚未接入真实硬件。**

支持的芯片规划：STM32F411 / F405 / F7 / H7，差异全部通过 `src/platforms/stm32/` 适配层解决，不复制 FlightCore。

## 构建

```bash
cmake -S . -B build/stm32 -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
    -DHOMEFLY_PLATFORM=stm32

cmake --build build/stm32
```

`cmake/arm-none-eabi.cmake` 是模板，按实际芯片调整 `-mcpu` / `-mfpu` / `-mfloat-abi` 与链接脚本。

## 移植清单

1. `src/platforms/stm32/stm32_clock.cpp/hpp` → `IClock`（DWT cycle counter 或 HAL tick）。
2. `stm32_gpio.cpp`、`stm32_timer.cpp`、`stm32_i2c.cpp`、`stm32_spi.cpp`、`stm32_uart.cpp` → 对应 HAL 接口。
3. 把 `src/platforms/stm32/logging/log_backend.cpp` 与 `flight_log_sink.cpp` 加入构建并定义 `HOMEFLY_PLATFORM_STM32`，板级提供 `hf_log_uart` 句柄；后续可换 ITM/SWO 或 `HAL_UART_Transmit_IT`。
4. BSP 提供 `hf_flight_log_mount/write/flush/close`：FatFs `f_mount` / `f_write` / `f_sync`（SD 卡）或 QSPI Flash，建议预分配连续文件 + DMA 双缓冲。
5. `src/applications/stm32/` 组装根，注入真实驱动、`FlightConfig`、`log_backend_console()`，并调用 `flight_log_storage_open()` + `FlightLogWriter::begin()`。
6. 用低优先级任务 / 空闲钩子同时调用 `log_drain(8)` 与 `flight_log.pump(recorder, 16)`，退出时 `finish()` + `flight_log_storage_close()`；控制循环内禁止任何同步输出或文件写入。
6. 启动文件、链接脚本、中断向量表放在板级目录，不进入 `src/flight_core/`。

## 约束

- 使用 ARM GNU Toolchain 交叉编译，公共模块仍由 CMake 管理。
- 平台层可实现 `IClock` / `IPwmOutput` 等，但不得反向依赖 FlightCore。
- 新增文件全部小写 snake_case。
