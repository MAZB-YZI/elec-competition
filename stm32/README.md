# Programm 目录说明

本目录用于存放电子设计竞赛相关 STM32 工程。当前主要是 STM32F407 HAL/CubeMX/Keil MDK 项目。

## 工程索引

| 工程目录 | 说明 | 关键入口 |
| --- | --- | --- |
| `ADC/` | ADC 采样基础工程 | `ADC.ioc`、`MDK-ARM/ADC.uvprojx` |
| `ADC vision/` | ADC 精度与信号识别相关工程 | `ADC vision.ioc`、`MDK-ARM/ADC vision.uvprojx` |
| `B_Borad/` | B 板相关工程 | `B_Borad.ioc`、`MDK-ARM/B_Borad.uvprojx` |
| `BreathingLight/` | 呼吸灯基础工程 | `BreathingLight.ioc`、`MDK-ARM/BreathingLight.uvprojx` |
| `Encode/` | 编码器相关工程 | `Encode.ioc`、`MDK-ARM/Encode.uvprojx` |
| `F407 Hal/` | F407 HAL 综合工程，包含 A/B 板子项目 | `A_Board/A_Board.ioc`、`B_Borad/B_Borad.ioc` |
| `HCSR04Test/` | HC-SR04 超声波测距测试 | `HCSR04Test.ioc`、`MDK-ARM/HCSR04Test.uvprojx` |
| `MyDelay/` | 延时函数测试工程 | `MyDelay.ioc`、`MDK-ARM/MyDelay.uvprojx` |
| `PWM_Breath/` | PWM 呼吸灯工程 | `PWM_Breath.ioc`、`MDK-ARM/PWM_Breath.uvprojx` |
| `PWMcheck/` | PWM 检测/验证工程 | `PWMcheck.ioc`、`MDK-ARM/PWMcheck.uvprojx` |
| `Simple UART Tx/` | 串口发送基础工程 | `Simple UART Tx.ioc`、`MDK-ARM/Simple UART Tx.uvprojx` |
| `UART_TEST/` | 串口测试工程 | `UART_TEST.ioc`、`MDK-ARM/UART_TEST.uvprojx` |
| `stdemo/` | STM32 示例/模板工程 | `stdemo.ioc`、`MDK-ARM/stdemo.uvprojx` |

## 使用建议

1. 优先打开对应工程目录下的 `.ioc` 文件，用 STM32CubeMX 查看外设配置。
2. 使用 Keil MDK 时，打开 `MDK-ARM/*.uvprojx`。
3. 每个工程修改前，建议先在该工程目录新增简短记录，例如 `NOTES.md`，说明硬件连接、实验现象和关键参数。
4. 不建议直接移动已有工程目录。CubeMX 与 Keil 工程中可能包含相对路径，移动后可能导致编译或生成代码失败。

## 后续整理计划

- 为重点工程补充 `NOTES.md`
- 统一记录芯片型号、引脚分配、串口波特率、定时器频率等参数
- 清理已经被 `.gitignore` 覆盖的编译产物
- 将最终比赛工程单独标注为 `final` 或在 README 中置顶
