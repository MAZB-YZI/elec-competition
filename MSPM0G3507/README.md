# MSPM0G3507 Workspace

This directory contains the MSPM0G3507 competition workspace.

## Directory layout

```
MSPM0G3507/
├── Modules/                    # 模块库（可移植）
│   ├── BSP/                   # 板级支持包
│   │   ├── board.h
│   │   ├── delay.c/h
│   │   ├── key.c/h
│   │   └── led.c/h
│   ├── Drivers/               # 驱动层
│   │   ├── OLED/              # OLED 显示驱动
│   │   ├── F32C_MOTOR/        # F32C 无刷电机驱动
│   │   ├── DC_MOTOR/          # 直流电机驱动
│   │   └── UART_DEBUG/        # 调试串口驱动
│   └── Utils/                 # 工具库（待扩展）
├── Projects/                   # 应用层
│   ├── F32C_GIMBAL_MSPM0G3507/  # 云台项目
│   └── 10_DC_MOTOR_PID_3/      # 电机 PID 项目
├── Resources/                  # 资源文件（Git 忽略）
└── Docs/                       # 文档
```

## 模块说明

### BSP（板级支持包）

| 文件 | 用途 |
|------|------|
| board.h | 板级定义 |
| delay.c/h | 延时函数 |
| key.c/h | 按键驱动 |
| led.c/h | LED 驱动 |

### Drivers（驱动层）

| 模块 | 用途 | 适用范围 |
|------|------|----------|
| OLED | OLED 显示驱动（I2C） | 通用 |
| F32C_MOTOR | F32C 无刷电机驱动（UART3） | 专用 |
| DC_MOTOR | 直流电机驱动（PWM） | 专用 |
| UART_DEBUG | 调试串口驱动（UART0） | 通用 |

## 使用方式

### 方式1：引用模块路径（推荐）

在 `.cproject` 中添加 include path：
```
-I"${PROJECT_ROOT}/../../Modules/BSP"
-I"${PROJECT_ROOT}/../../Modules/Drivers/OLED"
-I"${PROJECT_ROOT}/../../Modules/Drivers/F32C_MOTOR"
```

### 方式2：复制模块到项目（简单）

```
新项目/
├── BSP/           # 从 Modules/BSP/ 复制
├── Drivers/       # 从 Modules/Drivers/ 复制需要的模块
└── main.c
```

## 新项目创建流程

1. **创建项目目录**
2. **从 Modules/ 复制需要的模块**
3. **配置 include path**
4. **开始写应用代码**

## Toolchain

- CCStudio: `D:/TI/CCS`
- MSPM0 SDK: `D:/TI/SDK/mspm0_sdk_2_10_00_04`
- CCS projects: `D:/TI/Projects/elec-competition/MSPM0G3507/Projects`
