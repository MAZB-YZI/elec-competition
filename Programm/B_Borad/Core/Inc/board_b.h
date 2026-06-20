#ifndef __BOARD_B_H__
#define __BOARD_B_H__

#include "main.h"
#include "protocol.h"

/* ── 硬件引脚说明（对应 CubeMX 配置）──
 *   PA0        : K2 按键（板载 WKUP，按下=高电平）
 *   PB2        : 板载 LED2，D2 状态指示（低有效）
 *   PB0        : TIM3_CH3  PWM → 外接 D2 亮度控制
 *   PE9        : TIM1_CH1  互补 PWM 正相输出
 *   PE8        : TIM1_CH1N 互补 PWM 反相输出
 *   PA9/PA10   : USART1 TX/RX → 与 A 板通信
 *   PB6/PB7    : I2C1 SCL/SDA → SSD1306 OLED
 *   PC0        : ADC1_IN10 → Vi 输入
 * ─────────────────────────────────────── */

/* TIM3 ARR = 839，PWM 频率 100kHz
 *   最小占空比(微光)：CCR = 42  ≈ 5%
 *   最大占空比：      CCR = 839 = 100%
 *   Vi=0→CCR_MIN_GLOW, Vi=3.3V→CCR=839 */
#define D2_CCR_MIN_GLOW  42u
#define D2_CCR_MAX       839u

/* TIM1 ARR = 33599，PWM 频率 5kHz
 *   10% → CCR = 3360
 *   50% → CCR = 16800
 *   90% → CCR = 30240  */
#define PWM_CCR_10PCT    3360u
#define PWM_CCR_50PCT    16800u
#define PWM_CCR_90PCT    30240u

/* 心跳超时：5 个心跳周期（100ms×5=500ms<1s，保证1s内检测到）*/
#define HEARTBEAT_TIMEOUT_MS   800u
/* 发送间隔 */
#define HEARTBEAT_TX_MS        200u

/* ADC 缓冲区大小（1kHz 采样，1s 窗口）*/
#define ADC_BUF_SIZE     1000u

/* 对外接口 ───────────────────────────── */
void BoardB_Init(void);
void BoardB_MainLoop(void);

/* 以下由中断回调调用 */
void BoardB_TIM6_1ms_Callback(void);   /* TIM6 PeriodElapsedCallback */
void BoardB_TIM7_100ms_Callback(void); /* TIM7 PeriodElapsedCallback */
void BoardB_UART_RxCallback(uint16_t size); /* UART IDLE 回调 */
void BoardB_ADC_ConvCallback(void);    /* ADC DMA 半传输/全传输回调（可选）*/

#endif /* __BOARD_B_H__ */
