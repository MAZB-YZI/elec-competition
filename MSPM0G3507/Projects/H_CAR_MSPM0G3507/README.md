# H_CAR_MSPM0G3507 自动行驶小车

对应 2024 年全国大学生电子设计竞赛模拟电子系统专题赛 H 题，题目位于真题汇总 PDF 第 232～234 页。

## 源码结构

```text
empty.c                 正式主函数
include/                公共接口
src/app/                初始化、安全状态与周期调度
src/control/            运动控制与比赛路线状态机
src/drivers/            电机、编码器、MPU6050、蜂鸣器及灰度接口桩
tests/                  PC 单元测试，不加入 CCS 正式目标
```

运行测试：`powershell -ExecutionPolicy Bypass -File tests/run_tests.ps1`。

## 题目路径

- 测试 1：A→B，直线 1.00 m，15 s 内停车。
- 测试 2：A→B 直线、B→C 右半圆、C→D 直线、D→A 左半圆，30 s 内完成一圈。
- 测试 3：A→C 对角线、C→B 右半圆、B→D 对角线、D→A 左半圆，40 s 内完成一圈。
- 测试 4：按测试 3 连续运行 4 圈。

半圆半径 0.40 m、弧长约 1.2566 m；A-B/C-D 为 1.00 m；A-C/B-D 对角线约 1.280625 m。小车只允许前进，不得后退或使用原地反向差速转向。

`route_fsm.c` 已提供四个测试模式。直线和对角线由编码器里程与 MPU6050 航向保持完成，半圆由灰度接口循迹。每到达一个端点触发蜂鸣器和 PB22 板载 LED，提示持续约 120 ms。

## 原理图确认后的引脚

| 功能 | 引脚 |
|---|---|
| Motor1 PWMA/AIN1/AIN2 | PA12 / PB17 / PB19 |
| Motor1 Encoder A/B | PA25 / PA14 |
| Motor2 PWMB/BIN1/BIN2 | PA13 / PA16 / PB24 |
| Motor2 Encoder A/B | PA26 / PA27 |
| I2C0 SDA/SCL | PA28 / PA31 |
| MPU6050 INT | PB4，可选 |
| Buzzer | PA7 |
| 声光提示 LED | PB22，开发板板载 LED |

TB6612 STBY 固定接 5 V，不占 MCU 引脚。PA16 不再作为旋钮 ADC，PA27 不再作为舵机 PWM。F32C_MOTOR 不属于本工程。

## 软件接入

`hcar_hal.h` 是唯一硬件绑定边界，必须使用正式工程 SysConfig 生成的准确宏实现。推荐调度：

- 1 ms：`Buzzer_Update1ms()`；
- 5 ms：`MPU6050_Update(0.005f)`；
- 10 ms：`Encoder_Update(0.01f)`、`Motion_Update10ms()`、`Route_Update()`。

灰度队友实现 `LineSensor_GetError/IsValid/IsEndpoint/IsLost`。默认弱桩报告无效/丢线，进入半圆循迹段时会安全停机。

本目录暂不手写 CCS 元数据、`.syscfg` 或生成文件，须通过 CCS Project/SysConfig 工具创建和验证正式工程。

`empty.c` 是正式应用入口：先调用生成的 `SYSCFG_DL_init()`，再安全初始化应用并启动 1 ms 时基。实际定时器 IRQ 名称由 SysConfig 决定，因此 IRQ 处理函数放在后续的 MSPM0 HAL 实现中，并调用 `HCar_1msTickCallback()`。
