#ifndef __BOARD_A_H__
#define __BOARD_A_H__

#include "main.h"
#include "protocol.h"

/* ── 硬件引脚说明 ──────────────────────────────────────
 *   PA0        : K1 按键（板载 WKUP，按下=高电平）
 *   PB0        : TIM3_CH3 PWM → 外接 D1 呼吸灯
 *   PB2        : 板载 LED2（调试指示用）
 *   PC0        : ADC1_IN10 → Vi 输入
 *   PA9/PA10   : USART1 TX/RX → 与 B 板通信
 *   PB6/PB7    : I2C1 SCL/SDA → SSD1306 OLED
 * ──────────────────────────────────────────────────── */

/* TIM3 ARR = 839，PWM 频率 100kHz
 *   D1 呼吸灯用软件 PWM（TIM6 中断步进 CCR）
 *   CCR 范围：0 ~ 839 */
#define D1_CCR_MAX       839u

/* 呼吸灯三种半周期（单位：ms，即 TIM6 中断次数）
 *   T1=0.5s → 半周期 250ms
 *   T2=1s   → 半周期 500ms
 *   T3=2s   → 半周期 1000ms */
#define BREATH_HALF_T1   250u
#define BREATH_HALF_T2   500u
#define BREATH_HALF_T3   1000u

/* 心跳超时 */
#define HEARTBEAT_TIMEOUT_MS   800u
#define HEARTBEAT_TX_MS        200u

/* ADC 缓冲区（1kHz 采样，1s 窗口）*/
#define ADC_BUF_SIZE     1000u

/* 信号类型 */
typedef enum {
    SIG_NONE   = 0,   /* 无输入 */
    SIG_DC     = 1,   /* 直流   */
    SIG_SINE   = 2,   /* 正弦波 */
    SIG_SQUARE = 3,   /* 方波   */
} SigType_t;

/* 对外接口 */
void BoardA_Init(void);
void BoardA_MainLoop(void);

void BoardA_TIM6_1ms_Callback(void);
void BoardA_TIM7_100ms_Callback(void);
void BoardA_UART_RxCallback(uint16_t size);

#endif /* __BOARD_A_H__ */
