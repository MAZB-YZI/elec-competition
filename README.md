# elec-competition

湖南大学 2026 省电赛电子设计竞赛项目集。

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
│   ├── Liner_Car0_Tom/            # 巡线小车工程（队友版）
│   └── Liner_Car0_Rebuilt/        # 巡线小车工程（重构版）⭐
├── ec_vision/                     # 视觉识别方案（Python）
├── stm32/                         # STM32 旧工程（已废弃）
├── training/                      # 训练代码
└── Past Exam Questions/           # 历年电赛真题（2017-2025）
```

## 主要项目

### Liner_Car0_Rebuilt（巡线小车重构版）⭐

巡线小车主控工程，支持 PID 巡线、直角转弯、蓝牙调参。

**功能特性：**
- 8 路灰度传感器 + PD 巡线控制
- 陀螺仪角度环 + 编码器距离判断的直角转弯
- 速度环框架（默认关闭，可启用）
- 90° 角度环 PID（转弯自动减速）
- 编码器方向归一化（支持反向安装）
- 全局累计圈数显示
- HC-05 蓝牙实时调参
- OLED 实时显示状态（编码器速度、圈数、航向）
- 全白/全黑超时停车保护

### ec_vision（视觉识别）

Python 视觉方案，用于目标识别和云台控制。

### Past Exam Questions（历年真题）

2017-2025 年全国大学生电子设计竞赛真题汇总，用于备赛参考。

## 共享模块更新

### 2025-07-16 驱动同步

从 `Liner_Car0_Rebuilt` 同步优化到 `Modules/Drivers/`：

| 文件 | 更新内容 |
|------|----------|
| `DC_MOTOR/motor.c/h` | 添加 PID 控制器、方向缓存优化 |
| `OLED/oled.c/h` | 添加脏页优化，提高刷新效率 |

**保留 Modules 版本的文件：**
- `JY61P/jy61p.c/h` — 功能更全（错误统计、ResetYawTo 等）
- `GraySensor/gray_sensor.c/h` — 差异很小

## 硬件平台

- **MCU**：MSPM0G3507（LQFP-64）
- **开发板**：LCK-FB-TMX-MSPM0G3507（天猛星）
- **扩展板**：2026 电赛天猛星扩展板
- **电机驱动**：TB6612 双路 H 桥
- **编码器**：JGB37-520 霍尔编码器（AB 相）
- **陀螺仪**：JY61P（UART，WIT 协议）
- **灰度传感器**：8 路模拟灰度
- **显示**：0.96 寸 OLED（SSD1306，I2C）
- **蓝牙**：HC-05（UART，调参用）

## 分支说明

- `main`：主分支，包含所有项目代码

## 使用说明

1. 克隆仓库：`git clone https://github.com/MAZB-YZI/elec-competition.git`
2. 用 CCS（Code Composer Studio）打开 `MSPM0G3507/` 下的工程
3. 编译、烧录、调试

## 注意事项

- 共享模块在 `Modules/` 目录，各项目通过桥接文件引用
- 修改共享模块前先和队友沟通
- 提交前先 `git pull` 拉取最新代码
