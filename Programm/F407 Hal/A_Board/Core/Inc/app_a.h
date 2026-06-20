#ifndef __APP_A_H__
#define __APP_A_H__

#include "main.h"

/* ================================================================
 * Signal Type Enums
 * ================================================================ */
typedef enum {
    SIGNAL_NONE   = 0,
    SIGNAL_DC     = 1,
    SIGNAL_SINE   = 2,
    SIGNAL_SQUARE = 3
} SignalType_t;

/* B board PWM mode (from B via communication) */
typedef enum {
    PWM_MODE_FOLLOW = 0,   /* Duty follows Vi */
    PWM_MODE_MAX    = 1,   /* Duty = max Vi in past 1s */
    PWM_MODE_MIN    = 2    /* Duty = min Vi in past 1s */
} PWMMode_t;

/* Communication state */
typedef enum {
    COMM_OK   = 0,
    COMM_LOST = 1
} CommState_t;

/* ================================================================
 * Global System State
 * ================================================================ */
typedef struct {
    /* --- Signal Analysis --- */
    SignalType_t signal_type;    /* Detected signal type */
    float        voltage;        /* DC voltage or AC amplitude (V) */
    float        vi_instant;     /* Most recent instantaneous Vi (V) */

    /* --- D1 LED (local, PWM breathing on PA5) --- */
    uint8_t d1_enabled;          /* 1 = on (breathing), 0 = off */
    float   breath_phase;        /* Current phase [0, 2π) */

    /* --- D2 LED (B board, known via communication) --- */
    uint8_t d2_enabled;

    /* --- PWM mode (from B board) --- */
    PWMMode_t pwm_mode;

    /* --- Communication --- */
    CommState_t comm_state;
    uint32_t    last_rx_tick;    /* HAL_GetTick() of last valid B packet */
} SystemState_t;

extern SystemState_t g_state;

/* ================================================================
 * Public API
 * ================================================================ */
void AppA_Init(void);
void AppA_Process(void);         /* Called from main loop */
void AppA_Tick1kHz(void);        /* Called from TIM3 ISR @ 1kHz */
void AppA_EXTI2_Callback(void);  /* Called from EXTI2 IRQ (K1 press) */
void AppA_UART_RxCplt(uint8_t byte); /* Called from UART RX callback */

#endif /* __APP_A_H__ */
