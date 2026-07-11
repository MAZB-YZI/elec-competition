# H_CAR_MSPM0G3507 自动行驶小车

对应 2024 年全国大学生电子设计竞赛模拟电子系统专题赛 H 题“自动行驶小车”。当前主控为 TI MSPM0G3507，开发板为立创·天猛星 MSPM0G3507。

## 开发约束

- 本工程优先复用仓库 `Modules/` 下已有代码，不从 0 重写已有驱动。
- 可复用驱动的真实实现应放在 `Modules/Drivers/`；项目目录只保留板级适配、应用调度、测试入口和路线逻辑。
- 如果已有模块不符合实际硬件，以“修改共享模块 + 项目板级适配文件”的方式处理，避免复制一份同名私有驱动。
- `Modules/Drivers/F32C_MOTOR` 是已完成的云台无刷电机驱动，不属于本小车工程；没有用户明确同意时不要修改，也不要加入 H_CAR 编译。
- 灰度传感器由队友实现，本工程只依赖抽象接口，不假设 ADC/GPIO/串口/数据格式。
- `.syscfg`、`ti_msp_dl_config.c/h`、`.project`、`.cproject`、`.ccsproject` 和 `Debug/Release` 生成物必须通过 CCS/SysConfig/CCS Project 工具维护，不手工改。
- 硬件验证必须逐步报告：源码检查、SysConfig 检查、CCS 编译、烧录、实物行为是不同结论。

## 当前源码结构

```text
empty.c                 当前恢复安全入口：只闪烁 PB22 板载 LED
include/                H_CAR 公共接口和模块转发头
src/app/                初始化、安全状态、自测试入口与周期调度
src/control/            运动控制与路线状态机
src/drivers/            H_CAR 板级适配文件、编码器、蜂鸣器、灰度接口桩、模块桥接文件
tests/                  PC 单元测试，不加入 CCS 正式目标
```

当前 `motor.c` 和 `mpu6050.c` 是临时 CCS 桥接文件：

- `src/drivers/motor.c` 编译 `Modules/Drivers/DC_MOTOR/motor.c`
- `include/motor.h` 转发到 `Modules/Drivers/DC_MOTOR/motor.h`
- `src/drivers/mpu6050.c` 编译 `Modules/Drivers/MPU6050/mpu6050.c`
- `include/mpu6050.h` 转发到 `Modules/Drivers/MPU6050/mpu6050.h`

这样做是为了在不手工编辑 `.cproject` 的前提下，让 H_CAR 先复用共享模块。后续应在 CCS GUI 中把这些桥接文件替换为正式 linked source/include path。

## 已确认引脚

| 功能 | 引脚 |
|---|---|
| Motor1 PWMA / AIN1 / AIN2 | PA12 / PB17 / PB19 |
| Motor1 Encoder A / B | PA25 / PA14 |
| Motor2 PWMB / BIN1 / BIN2 | PA13 / PA16 / PB24 |
| Motor2 Encoder A / B | PA26 / PA27 |
| OLED 硬件 I2C0 SDA / SCL | PA28 / PA31 |
| MPU6050 软件 I2C SDA / SCL | PA0 / PA1 |
| MPU6050 INT | PB4，可选 |
| Buzzer | PA7 |
| 声光提示 LED | PB22，开发板板载 LED |

TB6612 STBY 固定接高电平，不占 MCU 引脚。PA16 不再作为旋钮 ADC，PA27 不再作为舵机 PWM。TB6612 逻辑电平按模块确认可接受 0-5 V，但 I2C 上拉必须保持 3.3 V，不能把 MSPM0 引脚拉到 5 V。

注意：扩展板 OLED 接口和 MPU6050 接口不是同一对物理 I2C 线。OLED 座使用 PA28/PA31，MPU6050 座使用 PA0/PA1；二者虽然都可作为 I2C0 复用引脚，但同一个硬件 I2C0 只能选择其中一组。正式小车工程不飞线：OLED 使用硬件 I2C0，MPU6050 保持 PA0/PA1 并改用软件 I2C。PA0/PA1 在正式工程中应作为 GPIO 管理，不得再配置为 I2C0 外设引脚。

## 路线目标

- 测试 1：A→B，直线 1.00 m，5 s 内停车。
- 测试 2：A→B 直线、B→C 右半圆、C→D 直线、D→A 左半圆，30 s 内一圈。
- 测试 3：A→C 对角线、C→B 右半圆、B→D 对角线、D→A 左半圆，40 s 内一圈。
- 测试 4：按测试 3 连续运行 4 圈。

几何参数：半圆半径 0.40 m，弧长约 1.2566 m；A-B/C-D 为 1.00 m；A-C/B-D 对角线约 1.280625 m。小车只允许前进，不得后退或使用原地反向差速转向。

## 软件接口分层

`hcar_hal.h` 是项目和硬件的边界，也就是“板级适配文件”的接口。共享模块不得直接依赖 H_CAR 的 SysConfig 宏，而是通过平台 hook 访问硬件：

- DC motor 模块调用 `Motor_PlatformSetDirection()` / `Motor_PlatformSetDuty()`。
- MPU6050 模块调用 `MPU6050_PlatformWrite()` / `MPU6050_PlatformWriteRead()`。
- H_CAR 的 `hcar_hal.c` 负责把这些 hook 映射到当前 SysConfig 生成的 GPIO/I2C/PWM 宏。

推荐调度周期：

- 1 ms：`Buzzer_Update1ms()`
- 5 ms：`MPU6050_Update(0.005f)`
- 10 ms：`Encoder_Update(0.01f)`、`Motion_Update10ms()`、`Route_Update()`

灰度队友实现以下接口即可：

- `LineSensor_GetError()`
- `LineSensor_IsValid()`
- `LineSensor_IsEndpoint()`
- `LineSensor_IsLost()`

默认灰度桩会报告无效/丢线，进入半圆循迹阶段时应安全停机。

## 测试顺序

当前已验证：Factory Reset 恢复后，安全版 `empty.c` 可下载，PB22 板载 LED 正常闪烁。

后续不要一次性烧完整路线程序，按以下顺序逐步启用：

1. LED/GPIO：PB22 闪烁。
2. 蜂鸣器：PA7 短响，先不进入运动控制。
3. 电机方向 GPIO：车轮架空或断开电机电源，仅测 AIN/BIN 电平。
4. PWM：PA12/PA13 低占空比输出，车轮架空。
5. 编码器：只读 A/B 相计数，不开闭环。
6. I2C/MPU6050：先 WHO_AM_I，再校准零偏和 yaw 积分。
7. 1 ms tick 和控制环：确认所有 handler 存在且 ISR 短小后再启用。
8. 路线状态机：先直线，再半圆接口，最后四圈。

## 当前注意事项

- `empty.c` 目前是恢复安全入口，不启动 H_CAR 调度。
- 当前板级适配文件中 PWM duty 仍是占位逻辑；正式电机 PWM 需要通过 SysConfig/CCS Project 工具配置并生成正确宏后接入。
- CCS 工程元数据中仍可能有旧工程名残留，不能手工改 `.cproject`；需要用 CCS 工程工具修正或重建工程。
