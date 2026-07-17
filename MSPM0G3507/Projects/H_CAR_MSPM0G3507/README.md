# H_CAR_MSPM0G3507 自动行驶小车

2024 年全国大学生电子设计竞赛 H 题"自动行驶小车"。主控 TI MSPM0G3507，开发板为立创·天猛星 MSPM0G3507。

## 项目结构

```text
include/                公共头文件
src/control/            运动控制（control.c）与路线状态机（route_fsm.c）
src/drivers/            板级适配：电机、编码器、灰度、陀螺仪、蜂鸣器、蓝牙
main.c                  测试入口（宏切换测试模式）
empty.syscfg            SysConfig 配置文件
```

## 测试模式

通过 `main.c` 开头的宏切换：

| 宏 | 功能 |
|---|---|
| `TEST_SPEED_LOOP` | 定速直行测试（速度环 PID） |
| `TEST_DIST_MODE` | 定距 1 米直行停车（A→B 模拟） |
| `TEST_LINE_MODE` | 半圆弧循迹测试（灰度 + 蓝牙调参） |

## 引脚分配

| 功能 | 引脚 | SysConfig 宏 |
|---|---|---|
| 左电机 PWM | PA12 | `PWM_MOTOR_C0` |
| 右电机 PWM | PA13 | `PWM_MOTOR_C1` |
| 左电机方向 | PB17 (DIR1), PB19 (DIR2) | `MOTOR_DIR_L`, `MOTOR_DIR_L2` |
| 右电机方向 | PA16 (DIR1), PB24 (DIR2) | `MOTOR_DIR_R`, `MOTOR_DIR_R2` |
| 左编码器 A/B | PA26 / PA27 | `ENCODER1_A`, `ENCODER1_B` |
| 右编码器 A/B | PA25 / PA14 | `ENCODER2_A`, `ENCODER2_B` |
| 灰度 CLK/DAT | PB6 / PB7 | `GRAY_SENSOR` |
| JY61P 陀螺仪 | UART0, PA0(TX) / PA1(RX) | `UART_JY61P` |
| HC-05 蓝牙 | UART3, PB2(TX) / PB3(RX) | `UART_BT` |
| OLED | I2C0, PA28(SDA) / PA31(SCL) | `I2C_OLED` |
| 蜂鸣器 | PA7 | `BUZZER`（低电平有效） |
| LED 状态灯 | PB22 | `LED_STATUS` |
| UART 调试 | UART1, PB6(TX) / PB7(RX) | `UART_DEBUG` |

## 控制架构

### 控制模式

| 模式 | 活跃的环 | 说明 |
|---|---|---|
| MOTION_LINE（循迹） | 1个：循迹 PD | 固定 PWM + 转向，参考 Liner_Car0_Rebuilt |
| MOTION_DISTANCE（直行） | 2个：速度 PI + 航向 PD | 编码器测距 + JY61P 保持直线 |
| MOTION_HEADING（定航向） | 2个：速度 PI + 航向 PD | 指定目标航向 |

### 循迹算法

完全移植自 `Liner_Car0_Rebuilt`：

- **加权位置**：`200*s[0] + 140*s[1] + 75*s[2] + 40*s[3] - 40*s[4] - 75*s[5] - 140*s[6] - 200*s[7]`
- **死区**：±3
- **PD 控制**：KP=1.8, KD=0.0
- **转向限速**：75/周期
- **输出**：`steer = -(KP×pos + KD×d_pos)`，范围 -1000~+1000

### 蓝牙调参（HC-05, 38400 波特率）

| 命令 | 功能 | 示例 |
|---|---|---|
| `hello` | 通信测试 | 返回 `receive:hello` |
| `SHOW` | 显示当前参数 | `LKP=1.80 LKD=0.00 BASE=600 HKP=1 SKP=1500` |
| `LKP x.x` | 设置循迹比例系数 | `LKP 1.5` |
| `LKD x.x` | 设置微分系数 | `LKD 0.1` |
| `BASE xxxx` | 设置循迹基准 PWM | `BASE 500` |
| `HKP xxxx` | 设置航向环比例系数 | `HKP 2` |
| `SKP xxxx` | 设置速度环比例系数 | `SKP 1000` |

手机 APP 需支持 ASCII 发送，不需要自动加换行符（代码有 200ms 超时处理）。

## 路线参数

| 段 | 类型 | 距离 |
|---|---|---|
| A→B / C→D | 直线 | 1.000 m |
| A→C / B→D | 对角线 | 1.280625 m |
| B→C / D→A / C→B | 半圆弧 | 弧长 1.2566 m（半径 0.40 m） |

## 共享模块依赖

| 模块 | 路径 | 用途 |
|---|---|---|
| Encoder | `Modules/Drivers/Encoder/` | 编码器计数 + 方向归一化 |
| GraySensor | `Modules/Drivers/GraySensor/` | 8 路灰度串行移位读取 |
| Buzzer | `Modules/Drivers/Buzzer/` | 蜂鸣器定时控制 |
| JY61P | `Modules/Drivers/JY61P/` | UART 陀螺仪解析 |
| MPU6050 | `Modules/Drivers/MPU6050/` | 软件 I2C 陀螺仪（备用） |
| PID | `Modules/Control/` | PID 控制器 |
| Bluetooth | `Modules/Drivers/Bluetooth/` | 通用蓝牙 UART 基础设施 |

## 开发约束

- 优先复用 `Modules/` 下已有代码，不从 0 重写。
- `.syscfg`、`.project`、`.cproject` 通过 CCS/SysConfig 工具维护，不手工改。
- H 题规则：只能前进，不能后退；不能遥控；不能用摄像头。
- 灰度传感器引脚 PB6/PB7，极性：DAT 低电平 = 黑线 = bit1。
- 蜂鸣器低电平有效，SysConfig 初始值 Set（高电平 = 不响）。
