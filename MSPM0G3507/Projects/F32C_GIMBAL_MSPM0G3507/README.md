# F32C_GIMBAL_MSPM0G3507

MSPM0G3507 双轴无刷云台电机驱动，基于 WHEELTEC F32C TTL 无刷电机。

## 功能特性

- 双电机独立控制（YAW + PITCH）
- 多圈位置闭环控制
- 上电软件零点
- 永久零点保存（EEPROM）
- 回到零位功能
- 上位机串口命令控制
- OLED 状态显示（I2C）
- 按键控制（单击/双击/长按）

## 硬件接线

### F32C 无刷电机

| 扩展板 | F32C电机 | 说明 |
|--------|----------|------|
| 12V | VIN/12V | 电机供电 |
| GND | GND | 公共地 |
| PB2 (UART3 TX) | RX | 电机通信 |
| PB3 (UART3 RX) | TX | 电机通信 |

两个电机并联在同一总线上，通过 ID 区分。

### USB-TTL 调试串口（上位机通信）

| USB-TTL | 扩展板 | 说明 |
|---------|--------|------|
| RXD | PA10 | UART0 TX |
| TXD | PA11 | UART0 RX |
| GND | GND | 公共地 |

### OLED（I2C）

| OLED | 扩展板 | 说明 |
|------|--------|------|
| VCC | 5V | 供电 |
| GND | GND | 地 |
| SCL | PA31 | I2C0 SCL |
| SDA | PA28 | I2C0 SDA |

### 其他外设

| 功能 | 引脚 | 说明 |
|------|------|------|
| 按键 | PA18 | 单击/双击/长按 |
| LED | PB9 | 状态指示 |

## 上位机串口命令

串口设置：**115200 8N1**

### 基础控制命令

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| `ENABLE` | `ENABLE` | 使能两个电机 | `ENABLE` |
| `DISABLE` | `DISABLE` | 失能两个电机 | `DISABLE` |
| `STATUS` | `STATUS` | 查询当前位置和速度 | `STATUS` |
| `SCAN` | `SCAN` | 扫描电机地址 | `SCAN` |

### 位置控制命令（单位：度）

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| `M1POS` | `M1POS<角度>` | 设置 YAW 轴目标位置（相对零点） | `M1POS90` |
| `M2POS` | `M2POS<角度>` | 设置 PITCH 轴目标位置（相对零点） | `M2POS45` |
| `M1ADD` | `M1ADD<角度>` | YAW 轴增加角度 | `M1ADD10` |
| `M1SUB` | `M1SUB<角度>` | YAW 轴减少角度 | `M1SUB5` |
| `M2ADD` | `M2ADD<角度>` | PITCH 轴增加角度 | `M2ADD20` |
| `M2SUB` | `M2SUB<角度>` | PITCH 轴减少角度 | `M2SUB10` |

### 速度控制命令（单位：RPM）

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| `M1SPD` | `M1SPD<速度>` | 设置 YAW 轴速度 | `M1SPD30` |
| `M2SPD` | `M2SPD<速度>` | 设置 PITCH 轴速度 | `M2SPD20` |

### 模式控制命令

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| `M1MODE` | `M1MODE<模式>` | 设置 YAW 轴模式 | `M1MODE1` |
| `M2MODE` | `M2MODE<模式>` | 设置 PITCH 轴模式 | `M2MODE1` |

模式值：
- 0 = 速度闭环
- 1 = 多圈位置闭环（默认）
- 2 = 单圈位置闭环
- 3 = 多圈相对位置闭环
- 4 = 单圈相对位置闭环

### 零点控制命令

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| `ZERO` | `ZERO` | 把**当前位置**设为零点（临时，掉电丢失） | `ZERO` |
| `SAVE` | `SAVE` | 设置硬件零点并保存到 EEPROM（永久） | `SAVE` |
| `HOME` | `HOME` | 回到零位（会先使能电机） | `HOME` |

### MCU回复格式

```
YAW:目标=10°,当前=10°,速度=10RPM
PITCH:目标=5°,当前=5°,速度=10RPM
OK
```

## 使用流程

### 1. 赛场设零点流程（推荐）

```
断掉 12V 供电    ← 电机控制器完全断电
手动转到想用的角度
接通 12V 供电    ← 电机控制器重新初始化
等待 5 秒        ← 等待电机控制器初始化完成
上电自动设零点   ← MCU 读取电机当前位置
ENABLE          ← 使能电机
HOME            ← 回到零位
```

**说明**：这是赛场上调零位的推荐方法，不需要发任何命令，上电自动完成。

### 2. 设新零点流程（手掰方式）

```
断掉 12V 供电    ← 电机控制器完全断电
手动转到想用的角度
接通 12V 供电    ← 电机控制器重新初始化
等待 5 秒        ← 等待电机控制器初始化完成
上电自动设零点   ← MCU 读取电机当前位置
ENABLE          ← 使能电机
HOME            ← 回到零位
```

**说明**：与赛场流程相同，必须断电才能重新设零点。

### 3. 回到零位流程

```
ENABLE          ← 使能电机
HOME            ← 回到零位
```

### 4. 永久零位设置流程

```
ENABLE          ← 使能电机
M1POS200        ← YAW轴转到 200°
M2POS23         ← PITCH轴转到 23°
STATUS          ← 确认位置
ZERO            ← 把当前位置设为零点
STATUS          ← 确认：目标=0°,当前=0°
SAVE            ← 保存为永久零点
HOME            ← 回到零位
DISABLE         ← 失能电机
```

### 3. 步进调整流程

```
ENABLE          ← 使能电机
M1ADD10         ← YAW轴 +10°
STATUS          ← 确认位置
M1SUB5          ← YAW轴 -5°
STATUS          ← 确认位置
M2ADD20         ← PITCH轴 +20°
STATUS          ← 确认位置
DISABLE         ← 失能电机
```

## 上电流程

1. 使能电机
2. 设置多圈位置闭环模式
3. 设置速度（默认10RPM）
4. 读取当前位置作为零点
5. 所有角度命令相对于零点

## 零点设置说明

### 1. 上电自动设零位 ✅

MCU 复位/上电时会自动执行 `Gimbal_SetPowerOnZero()`，读取电机当前位置作为软件零点。

```c
/* 4. 上电自动设零点（失能状态下可以设零点） */
if (Gimbal_SetPowerOnZero()) {
    UART_Debug_SendString("Zero point set OK\r\n");
} else {
    UART_Debug_SendString("Zero point timeout!\r\n");
}
```

### 2. 任何状态 ZERO 也会设置零位 ✅

任何时候发 `ZERO` 命令都会设置零位，不需要使能电机。

```c
case DBG_CMD_ZERO:
    /* 先请求位置反馈，等待更新后再设置零点 */
    BLDC_ReqFeedback(motor1_ID, FB_MULTI_ANGLE);
    delay_ms(100);
    BLDC_ReqFeedback(motor2_ID, FB_MULTI_ANGLE);
    delay_ms(100);
    /* 设置零点 */
    motor1_zero_offset = Motor1_Current_Position;
    motor2_zero_offset = Motor2_Current_Position;
    Motor1_T_Position = 0;
    Motor2_T_Position = 0;
    break;
```

### 3. 总结

| 场景 | 是否自动设零位 | 说明 |
|------|---------------|------|
| **MCU 上电/复位** | ✅ 是 | 自动执行 `Gimbal_SetPowerOnZero()` |
| **发 ZERO 命令** | ✅ 是 | 任何时候都可以发，不需要使能电机 |

### 4. 推荐流程

```
DISABLE         ← 失能电机
                ← 手掰到想要的位置
ZERO            ← 设零位（任何状态都可以）
ENABLE          ← 使能电机
HOME            ← 回到零位
SAVE            ← 保存到 EEPROM（永久）
DISABLE         ← 失能电机
```

## 协议说明

帧格式：`0x7A` + 地址 + 命令 + 数据 + BCC校验 + `0x7B`

主要命令：
- `0x06` - 使能电机
- `0x05` - 失能电机
- `0x00` - 设置模式
- `0x01` - 设置速度
- `0x02` - 设置多圈位置
- `0x0E` - 请求反馈
- `0x0D` - 设置零点

## 目录结构

```
F32C_GIMBAL_MSPM0G3507/
├── empty.c              # 主程序
├── empty.syscfg         # SysConfig配置
├── Control/
│   ├── DataScope_DP.c/h # F32C协议层
│   ├── uart_callback.c/h # UART3电机通信
│   ├── uart_debug.c/h   # UART0上位机通信
│   ├── control.c/h      # 定时器控制
│   └── show.c/h         # OLED显示
├── drivers/
│   ├── oled.c/h         # OLED驱动(I2C)
│   ├── key.c/h          # 按键驱动
│   ├── led.c/h          # LED驱动
│   ├── delay.c/h        # 延时函数
│   └── board.h          # 板级定义
└── targetConfigs/
```

## 注意事项

1. 电机地址需要通过厂商软件设置（YAW=1, PITCH=2）
2. 设置地址后需要断电重启
3. OLED 使用 I2C 接口（PA28/PA31），不是 SPI
4. 默认速度10RPM，可通过 `M1SPD` / `M2SPD` 调整
5. `ZERO` 是临时零点（掉电丢失），`SAVE` 是永久零点
6. `HOME` 命令会先使能电机，再回到零位
7. `HOME` 命令后需要等待 3-5 秒再发 `STATUS`
8. **失能状态下可以设零点**：不需要先使能电机，手掰到位置后直接发 `ZERO`
9. **重新设零点必须断电**：电机不断电只失能，RESET 无法更新零点，必须断掉 12V 供电再重新上电

## 常见问题

### Q1: 发送命令没有响应
**A**: 检查串口设置（115200bps, 8N1）和接线（TX/RX是否交叉）

### Q2: 电机不转动
**A**: 先发 `ENABLE` 使能电机，再发位置命令

### Q3: 位置不准确
**A**: 先发 `ZERO` 设零点，再发位置命令

### Q4: 回到零位不准确
**A**: `HOME` 后等待 3-5 秒再发 `STATUS`

### Q5: 重新上电后位置丢失
**A**: 用 `SAVE` 保存为永久零点，而不是 `ZERO`

## 版本记录

- V1.0 - 基础电机控制
- V1.1 - 添加上位机命令
- V1.2 - 添加上电零点功能
- V1.3 - 修复角度显示问题
- V1.4 - 修复零点显示问题
- V1.5 - 修复目标位置重置问题
- V1.6 - 添加 HOME 命令，优化 ZERO 命令
- V1.7 - 添加失能状态设零点功能，优化流程
- V1.8 - 添加断电设零点流程，更新文档
