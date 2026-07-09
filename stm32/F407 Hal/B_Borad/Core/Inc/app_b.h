#ifndef __APP_B_H__
#define __APP_B_H__

#include "main.h"

/* ================================================================
 * Enums
 * ================================================================ */
typedef enum {
    PWM_MODE_FOLLOW = 0,   /* Duty follows Vi real-time */
    PWM_MODE_MAX    = 1,   /* Duty = max Vi in past 1s */
    PWM_MODE_MIN    = 2    /* Duty = min Vi in past 1s */
} PWMMode_t;

typedef enum {
    COMM_OK   = 0,
    COMM_LOST = 1
} CommState_t;

/* ================================================================
 * Global System State
 * ================================================================ */
typedef struct {
    /* --- Vi from A board via USART --- */
    float    vi_voltage;       /* Latest instantaneous Vi (V) — follow mode + D2 */
    float    vi_max;           /* 1 s peak Vi from A board (V) — MAX mode */
    float    vi_min;           /* 1 s trough Vi from A board (V) — MIN mode */

    /* --- D2 LED (local, TIM2 CH1 on PA5) --- */
    uint8_t  d2_enabled;       /* 1 = on, 0 = off (controlled by K1 long press from A) */
    float    d2_duty;          /* 0.0–1.0, updated every 1ms */

    /* --- D1 LED (A board, remote status) --- */
    uint8_t  d1_enabled;

    /* --- PWM mode (K2 cycles through 0→1→2→0) --- */
    PWMMode_t pwm_mode;

    /* --- Complementary PWM (TIM1) duty --- */
    float    pwm_duty;         /* 0.10–0.90 */

    /* --- Communication --- */
    CommState_t comm_state;
    uint32_t    last_rx_tick;  /* HAL_GetTick() of last valid A packet */
} SystemState_t;

extern SystemState_t g_state;

/* ================================================================
 * Public API
 * ================================================================ */
void AppB_Init(void);
void AppB_Process(void);          /* Called from main loop */
void AppB_Tick1kHz(void);         /* Called from TIM3 ISR @ 1kHz */
void AppB_EXTI2_Callback(void);   /* Called from EXTI2 IRQ (K2 press) */
void AppB_UART_RxCplt(uint8_t byte); /* Called from UART RX callback */

#endif /* __APP_B_H__ */
