#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

void BT_Init(void);
void BT_SetTickPtr(volatile uint32_t *tick_ptr);
bool BT_Poll(void);
void BT_Send(const char *str);
void BT_Printf(const char *fmt, ...);
void BT_SendParams(void);
void BT_Telemetry(uint8_t gray_raw, int16_t speed_l, int16_t speed_r, float yaw);
uint16_t BT_GetTelPeriod(void);

void UART_PB_INST_IRQHandler(void);

#endif /* __BLUETOOTH_H__ */
