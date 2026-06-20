# 电赛双板系统 — 设计思路与排查流程

## 一、题目要求概览

A 板：信号识别 + D1 呼吸灯 + 按键控制 + 通信
B 板：互补 PWM + D2 亮度跟随 + 按键切换模式 + 通信
核心难点：双板通信的可靠性（断线检测、状态同步）

---

## 二、硬件架构

```
信号发生器 ──→ PC0 (ADC) ──→ A板
                              │
                         UART 115200
                              │
                              ↓
                            B板 ──→ PWM输出 (PA8/PA7)
                              │
                         OLED (I2C)
```

### 引脚分配

| 功能 | A 板 | B 板 |
|------|------|------|
| Vi 输入 | PC0 (ADC1_IN10) | — |
| K1 按键 | PA2 (EXTI2) | — |
| K2 按键 | — | PA2 (EXTI2) |
| D1 呼吸灯 | PA5 (TIM2_CH1) | — |
| D2 亮度 | — | PA5 (TIM2_CH1) |
| PWM1 | — | PA8 (TIM1_CH1) |
| PWM2 | — | PA7 (TIM1_CH1N) |
| UART TX | PA9 | PA9 |
| UART RX | PA10 | PA10 |
| OLED SCL | PB6 | PB6 |
| OLED SDA | PB7 | PB7 |

---

## 三、软件架构

### A 板模块划分

```
AppA_Init()          → 初始化
AppA_Process()       → 主循环（信号分析、超时检测、显示刷新）
AppA_Tick1kHz()      → 1kHz 中断（ADC采集、呼吸灯、通信发送、按键扫描）
AppA_UART_RxCplt()   → UART 接收解析
AppA_EXTI2_Callback() → 按键中断（空，实际在 Tick 里轮询）
```

### B 板模块划分

```
AppB_Init()          → 初始化
AppB_Process()       → 主循环（超时检测、显示刷新）
AppB_Tick1kHz()      → 1kHz 中断（D2亮度、PWM更新、通信发送）
AppB_UART_RxCplt()   → UART 接收解析
AppB_EXTI2_Callback() → K2 按键中断
```

---

## 四、核心算法

### 4.1 信号识别（A 板）

**方法：过零 + 斜率联合判定**

```
1. range < 0.05V → 无输入或直流
2. 统计过零次数（以均值为基准，50mV 死区防抖）
3. 统计斜率陡变次数（相邻点差值 > 200 ADC 计数）
4. 综合判断：
   - 过零>=10 或 陡变>=2 → 方波
   - 过零>=2 → 正弦波
   - 兜底 → 正弦波
```

**注意事项：**
- 中轴用 `(max+min)/2` 比均值更稳定（方波占空比不是50%时均值会偏）
- 死区太小会被噪声干扰，太大会漏检小信号
- 1kHz 采样对 1~5Hz 信号完全够用（每周期 200~1000 个点）

### 4.2 幅值计算（A 板）

```
幅值 = (Vmax - Vmin) / 2.0
直流 = 平均电压
```

### 4.3 呼吸灯（A 板）

```
频率由 B 板 PWM 模式决定：
  FOLLOW → 0.5Hz（周期 2s）
  MAX    → 1.0Hz（周期 1s）
  MIN    → 2.0Hz（周期 0.5s）

亮度 = 0.49 × (1 + sin(phase)) + 0.02
  → 范围 0.02~1.0，保证最暗时也有微光
```

### 4.4 PWM 占空比（B 板）

```
跟随模式：duty = 0.10 + (Vi/3.3) × 0.80  → 覆盖 10%~90%
最大值模式：duty = ViHistory_GetMax() 映射
最小值模式：duty = ViHistory_GetMin() 映射
```

---

## 五、通信协议设计

### 5.1 协议演进过程

**第一版（3字节）：**
```
B→A: [0xBB, flags, checksum]
问题：FOLLOW+D2off 时 flags=0x00，checksum=0xBB 与帧头碰撞
```

**第二版（4字节，加 seq）：**
```
B→A: [0xBB, seq, flags, checksum]
解决：checksum = 0xBB^seq^flags，seq 每次递增，不会碰撞
```

**第三版（加 D1 状态）：**
```
A→B: [0xAA, seq, flags, vi_h, vi_l, checksum]
flags: bit0=D2 toggle, bit1=D1 状态
B→A: [0xBB, seq, flags, checksum]
flags: bit0=D2 状态, bit1-2=PWM 模式
```

**第四版（加 seq 确认，最终版）：**
```
A→B: [0xAA, seq, flags, vi_h, vi_l, last_b_seq, checksum]  (7字节)
B→A: [0xBB, seq, flags, last_a_seq, checksum]               (5字节)

last_b_seq: A板确认收到的B板seq
last_a_seq: B板确认收到的A板seq
```

### 5.2 断线检测机制

**两种检测方式并行：**

| 方式 | 检测什么 | 触发条件 |
|------|---------|---------|
| 超时检测 | 有没有收到对方的包 | 超过 800ms 没收到 |
| seq 确认 | 对方有没有收到自己的包 | tx_seq - confirmed_seq > 15 |

**覆盖场景：**

| 断线位置 | 超时检测 | seq 确认 |
|---------|---------|---------|
| A→B（A的TX） | B 板触发 ✓ | A 板触发 ✓ |
| B→A（B的TX） | A 板触发 ✓ | B 板触发 ✓ |
| 都不断 | 不触发 ✓ | 不触发 ✓ |

**关键保护：**
- `comm_first_rx` 保护：上电时 seq 差值检测跳过，避免误判
- 阈值 15（750ms）+ 超时 800ms，都在 1 秒以内

### 5.3 调试中踩过的坑

| 问题 | 原因 | 解法 |
|------|------|------|
| FOLLOW 模式传不过去 | flags=0x00 时 checksum 与帧头碰撞 | 加 seq 字节 |
| 断线只有一边 LOST | 超时检测是单向的 | 加 seq 确认机制 |
| seq 确认死锁 | 两边互相报告 LOST，复联后无法恢复 | 不用确认位，用 seq 差值 |
| 上电误判 LOST | tx_seq 涨到阈值但还没收到回包 | 加 comm_first_rx 保护 |
| 上电 PWM 10% | memset 清零 vi_voltage=0 | 初始化 vi_voltage=1.65V |
| K1 长按爆闪 | btn_long_sent 被清零后又触发 | 加 btn_long_done 标志 |

---

## 六、开发顺序建议

### 第一阶段：单板功能（1-2天）

1. **A 板 ADC 采集** — 确认 DMA 循环采样正常
2. **A 板信号识别** — 接信号发生器，验证 DC/Sine/Square 判定
3. **A 板幅值显示** — 与示波器对比，误差 <0.1V
4. **A 板呼吸灯** — 三种频率切换
5. **B 板 PWM 输出** — 确认 5kHz、互补、死区
6. **B 板 D2 亮度** — 跟随 Vi 变化

### 第二阶段：通信（1天）

1. **基础通信** — A→B 发 Vi，B→A 发状态
2. **状态同步** — D1/D2 状态、PWM 模式互相显示
3. **失联检测** — 超时 + seq 确认

### 第三阶段：按键与联动（半天）

1. **K1 短按** — D1 开关
2. **K1 长按** — D2 远程控制
3. **K2 短按** — PWM 模式切换
4. **失联保持** — 断线后 PWM/LED 保持最后状态
5. **复联恢复** — 补发 pending 命令

### 第四阶段：测试与优化（半天）

1. 逐项对照评分表测试
2. 调阈值、调显示、调响应速度

---

## 七、常见问题排查

### ADC 采不到波形

```
检查：PC0 接线是否正确
检查：DMA 是否启动（HAL_ADC_Start_DMA）
检查：TIM2 是否触发 ADC（CubeMX TRGO 配置）
验证：OLED 显示 Vmax/Vmin，看是否有变化
```

### 信号识别不准

```
检查：采样率是否足够（1kHz 对 1~5Hz 够用）
检查：阈值是否匹配实际信号幅度
调试：OLED 显示过零次数和斜率陡变次数
调参：调整 SIG_THRESH_LSB、SQWAVE_RATIO
```

### 通信建立不起来

```
检查：UART 波特率是否一致（115200）
检查：TX/RX 是否交叉连接（A的TX→B的RX）
检查：GND 是否共地
调试：OLED 显示 Comm:OK/LOST
调试：用示波器看 TX 引脚有没有波形
```

### 断线检测不灵敏

```
检查：COMM_TIMEOUT_MS 是否合适（建议 800ms）
检查：seq 差值阈值是否合适（建议 15）
检查：comm_first_rx 保护是否加上
测试：拔线后计时，看多久显示 LOST
```

### 呼吸灯不亮

```
检查：D1 是否开启（K1 短按切换）
检查：TIM2 CH1 PWM 是否启动
检查：PA5 引脚配置是否正确
调试：OLED 显示 D1:ON/OFF 状态
```

### PWM 占空比不对

```
检查：TIM1 ARR 是否正确（33599 = 5kHz）
检查：CubeMX 死区配置（DTG）
检查：Vi 电压是否正确传递到 B 板
调试：OLED 显示 PWM 百分比
调试：示波器测量实际占空比
```

---

## 八、评分项速查表

| # | 项目 | 关键代码 | 分值 |
|---|------|---------|------|
| 1 | 初始状态 | `memset` + `vi_voltage=1.65f` | 5 |
| 2 | 直流误差≤0.05V | ADC 精度 | 5 |
| 3 | 直流误差≤0.01V | ADC 精度 | 5 |
| 4 | K1短按D1 | `Button_Tick` 短按逻辑 | 5 |
| 5 | K1长按 | `LONG_PRESS_MS=800` | 5 |
| 6 | K2切换模式 | `(mode+1)%3` | 5 |
| 7 | 按键不影响显示 | ISR/主循环分离 | 5 |
| 8 | D1呼吸T1=0.5s | `BREATH_FREQ_MODE2=2.0` | 5 |
| 9 | D1呼吸T2=1s | `BREATH_FREQ_MODE1=1.0` | 5 |
| 10 | D1呼吸T3=2s | `BREATH_FREQ_MODE0=0.5` | 5 |
| 11 | 信号类型识别 | `Signal_Analyze` | 5 |
| 12 | 幅值显示 | `Vpp/2` | 5 |
| 13 | 最大Vi | `vmax` 在1s窗口取 | 5 |
| 14 | 最小Vi | `vmin` 在1s窗口取 | 5 |
| 15 | D2跟随Vi | `ViToDuty(vi, 0.02, 0.9)` | 5 |
| 16 | D2微光 | `TIM2_DUTY_MIN=0.02` | 5 |
| 17 | 无输入D2微光+PWM50% | 需确认 | 5 |
| 18 | PWM 5kHz | TIM1 ARR=33599 | 5 |
| 19 | PWM死区 | CubeMX DTG配置 | 5 |
| 20 | 跟随模式10%~90% | `ViToDuty(vi, 0.1, 0.9)` | 5 |
| 21 | 最大值模式 | `ViHistory_GetMax` | 5 |
| 22 | 最小值模式 | `ViHistory_GetMin` | 5 |
| 23 | 显示对方状态 | flags bit0/bit1 | 5 |
| 24 | 断线双方LOST | 超时800ms + seq差值15 | 5 |
| 25 | 复联恢复 | 收到包立刻OK | 5 |
| 26 | D1呼吸跟随模式 | `Breath_Update` 读 pwm_mode | 5 |
| 27 | D2亮度跟随Vi | `UpdateD2` 读 vi_voltage | 5 |
| 28 | 失联保持 | `COMM_LOST` 时保持最后值 | 5 |
| 29 | 失联改模式复联生效 | pwm_mode 实时更新 | 5 |
| 30 | 失联改D2复联生效 | `d2_cmd_pending` 补发 | 5 |
