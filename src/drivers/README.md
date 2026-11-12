# Drivers

具体硬件驱动层：把芯片/外设转换成 HAL 定义的统一接口。更换传感器时，原则上只需要更换驱动实现，不应修改 FlightCore 和控制算法。

## 现有实现

| 文件 | 作用 |
| --- | --- |
| `motor/drv_pwm_motor.hpp/.cpp` | 把 `IPwmOutput`（脉冲宽度 µs）适配成 `IMotor`（归一化推力 [0,1]），带未解锁互锁 |
| `board/drv_board_led.h/.c` | C11 板级原语示例，展示 C 与 C++ 的分工边界（软件影子状态，由平台层映射到真实 GPIO） |

## 规划中的驱动

```
drivers/
├── imu/
│   ├── drv_mpu6050.cpp/.hpp
│   └── drv_icm42688.cpp/.hpp
├── motor/
│   ├── drv_dshot.cpp/.hpp
│   └── drv_pwm.cpp/.hpp      # 具体定时器 PWM 实现 IPwmOutput
└── sensors/
    └── drv_bmp280.cpp/.hpp
```

## 新增驱动的规则

1. 先确认 HAL 里已有对应的能力接口（`IImu`、`IBarometer`、`IPwmOutput`…）；没有则先补 HAL，不要驱动直连 FlightCore。
2. 文件名 `drv_<chip>.cpp/.hpp`，全部小写 snake_case。
3. 头文件不得暴露芯片寄存器、厂商 SDK 类型或平台实现细节。
4. 电机/电源类驱动必须实现失效安全：初始化/断臂时输出最小值。
5. 在 `test/drivers/` 加模拟硬件测试，并更新本目录的 `CMakeLists.txt`。
