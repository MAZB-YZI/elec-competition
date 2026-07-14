# elec-competition

湖南大学 2026 省电赛电子设计竞赛项目集（H题：智能巡线小车）。

## 团队成员

- MAZB-YZI（仓库所有者）
- 7om-mspm0-pid
- xixiluyaoyao

## 项目结构

```
elec-competition/
├── MSPM0G3507/                    # MSPM0G3507 主控平台
│   ├── Modules/                   # 共享驱动和控制模块
│   │   ├── Control/               # PID 控制算法
│   │   └── Drivers/               # 硬件驱动
│   │       ├── DC_MOTOR/          # TB6612 双电机驱动
│   │       ├── Encoder/           # 编码器驱动
│   │       ├── JY61P/             # JY61P 串口陀螺仪
│   │       ├── Buzzer/            # 蜂鸣器驱动
│   │       └── OLED/              # OLED 显示驱动
│   ├── Projects/                  # 工程项目
│   │   ├── H_CAR_MSPM0G3507/     # H题主线工程（速度环测试）
│   │   ├── JY61P_OLED_MSPM0G3507/ # JY61P 陀螺仪测试工程
│   │   ├── 10_DC_MOTOR_PID_3/    # 电机+编码器+PID 测试工程
│   │   └── F32C_GIMBAL_MSPM0G3507/ # 云台工程
│   └── Liner_Car0_Tom/            # 队友主线工程（巡线+直角+蓝牙调参）
├── stm32/                         # STM32 旧工程（已废弃）
└── Resources/                     # 硬件资料、烧录工具等
```

## 开发阶段

| 阶段 | 目标 | 状态 |
|------|------|------|
| 1. 底层单模块确认 | 确认每个模块独立工作 | ✅ 已完成 |
| 2. 小闭环测试 | 定速直行、定航向、直角转弯 | 🚧 进行中 |
| 3. 路线状态机 | 串成H题路线 | ⏳ 待开始 |
| 4. 正式H题联调 | 完整跑题 | ⏳ 待开始 |

## 当前进度

### 已完成
- ✅ 电机驱动（TB6612 双路 PWM）
- ✅ 编码器驱动（AB 相中断计数）
- ✅ PID 控制算法（速度环、转向环）
- ✅ JY61P 陀螺仪驱动（WIT 协议解析）
- ✅ OLED 显示
- ✅ 蓝牙调参（HC-05）

### 进行中
- 🚧 速度闭环调试（50 cm/s 目标）
- 🚧 JY61P 集成到巡线工程

### 待开始
- ⏳ 灰度传感器集成
- ⏳ 路线状态机
- ⏳ 四圈流程

## 硬件平台

- **MCU**：MSPM0G3507（LQFP-64）
- **开发板**：LCK-FB-TMX-MSPM0G3507（天猛星）
- **扩展板**：2026 电赛天猛星扩展板
- **电机驱动**：TB6612 双路 H 桥
- **编码器**：JGB37-520 霍尔编码器（AB 相）
- **陀螺仪**：JY61P（UART，WIT 协议）
- **显示**：0.96 寸 OLED（SSD1306，I2C）

## 分支说明

- `main`：主分支，包含所有共享模块和测试工程
- `PID_linerCar`：队友巡线工程分支
- `feature/h-car-firmware`：H题主线开发分支

## 使用说明

1. 克隆仓库：`git clone https://github.com/MAZB-YZI/elec-competition.git`
2. 用 CCS（Code Composer Studio）打开 `MSPM0G3507/Projects/` 下的工程
3. 编译、烧录、调试

## 注意事项

- 共享模块在 `Modules/` 目录，各项目通过桥接文件引用
- 修改共享模块前先和队友沟通
- 提交前先 `git pull` 拉取最新代码
- 硬件连接参考 `MSPM0G3507/电赛巡线小车硬件连接手册_V3.2.md`
