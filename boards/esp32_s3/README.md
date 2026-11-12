# ESP32-S3（当前主要开发平台）

**状态：预留适配位，尚未接入真实硬件。** Linux Native 已完成架构验证，本目录用于 Phase 2 落地。

## 目标

- 验证飞控核心架构在真实 MCU 上可运行。
- 验证 IMU 数据采集、姿态解算、PID 控制、电机输出。
- 实现微型无人机首次稳定飞行。

## 构建

使用 ESP-IDF 官方构建体系，不用根 CMake 生成固件：

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 移植清单

1. 在 `src/platforms/esp32/` 新增：
   - `esp32_clock.cpp/hpp` → `IClock`，用 `esp_timer_get_time()`。
   - `esp32_gpio.cpp/hpp`、`esp32_timer.cpp/hpp`、`esp32_i2c.cpp/hpp` 等，只实现 HAL 接口。
2. 在 `src/drivers/imu/` 新增 `drv_mpu6050.*` / `drv_icm42688.*`，实现 `IImu`，禁止把 ESP-IDF 类型暴露到接口里。
3. 在 `src/drivers/motor/` 新增 `drv_dshot.*` 或 `drv_pwm.*`，实现 `IMotor`。
4. 把 `src/platforms/esp32/logging/log_backend.cpp` 与 `flight_log_sink.cpp` 加入 ESP-IDF 组件 `SRCS`（`ESP_PLATFORM` 由 IDF 定义）；文本日志走 UART0 / USB-UART 桥。
5. BSP 提供四个 C 钩子 `hf_flight_log_mount/write/flush/close`：用 `esp_vfs_fat_sdmmc_mount()` 挂载 SD 卡并 `fopen(..., "wb")`，建议预分配连续文件；这四个函数**只允许被低优先级日志任务调用**。
6. 新增 `src/applications/esp32_s3/` 组装根与 `app_main()`：注入上述实现 + `default_quad_x_config()`，调用 `log_init(clock, log_backend_console(), config)`、`flight_log_storage_open(nullptr)`、`FlightLogWriter::begin(flight_log_storage(), header)`。
7. 创建低优先级日志任务（优先级低于控制任务）周期性调用 `log_drain(8)` 与 `flight_log.pump(recorder, 16)`；退出/断臂时 `flight_log.finish(recorder.dropped())` + `flight_log_storage_close()`。**控制循环内禁止调用 drain / pump / finish 或任何文件写入**。
8. ESP-IDF 组件通过 `idf_component_register(SRCS ... INCLUDE_DIRS ...)` 引用 `src/platforms/common/logging/` 与其它公共源码，不复制 FlightCore。

后续增加无线遥测时只需新增 `ILogSink` 实现并 `log_add_sink()`；换 SD 卡 / Flash 存储介质只改 BSP 钩子，`.hfl` 格式与 `tools/hfl_dump.py` 不变。上真机前需实测 SD 卡写入 WCET 与掉电完整性。

## 约束

- 板级差异通过 `FlightConfig` 和平台适配解决，不改核心代码。
- 所有新增文件使用小写 snake_case。
- 电机相关改动必须有台架（卸桨）验证与失效安全说明。
