# elec-competition

湖南大学大二学生 2026 省电赛电子设计竞赛项目集。

## 项目说明

本仓库用于整理电子设计竞赛相关代码、资料、实验记录和调试过程，方便后续复盘、协作与迭代。

## 目录结构

- `stm32/`：STM32 工程项目目录，详见 `stm32/README.md`
- `.gitignore`：Git 忽略规则

## 当前工程索引

- `ADC/`：ADC 采样基础工程
- `ADC vision/`：ADC 精度与信号识别相关工程
- `B_Borad/`：B 板相关工程
- `BreathingLight/`：呼吸灯基础工程
- `Encode/`：编码器相关工程
- `F407 Hal/`：F407 HAL 综合工程
- `HCSR04Test/`：HC-SR04 超声波测距测试
- `MyDelay/`：延时函数测试工程
- `PWM_Breath/`：PWM 呼吸灯工程
- `PWMcheck/`：PWM 检测/验证工程
- `Simple UART Tx/`：串口发送基础工程
- `UART_TEST/`：串口测试工程
- `stdemo/`：STM32 示例/模板工程

## 使用说明

1. 查看 `stm32/README.md` 选择对应工程。
2. 用 STM32CubeMX 打开工程目录下的 `.ioc` 文件查看配置。
3. 用 Keil MDK 打开 `MDK-ARM/*.uvprojx` 进行编译、下载和调试。

## 维护计划

- 整理各项目子目录说明
- 补充硬件连接与引脚定义
- 记录关键算法、调参过程和实验结果
