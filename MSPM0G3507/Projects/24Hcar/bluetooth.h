#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float   KP;
    float   KI;
    int16_t BASE_PWM;
    int16_t TURN_SPEED;
    int16_t OUTPUT_LIM;
} TuningParams_t;

void BT_Init(void);
void BT_SetTickPtr(volatile uint32_t *tick_ptr);
bool BT_Poll(TuningParams_t *params);
void BT_Send(const char *str);
void BT_Printf(const char *fmt, ...);
void BT_SendParams(const TuningParams_t *p);

void UART_PB_INST_IRQHandler(void);

#endif /* __BLUETOOTH_H__ */
