/**
 * jy61p.h — JY61P UART IMU driver
 *
 * UART0: PA0=TX, PA1=RX, 9600 baud.
 * Parses WIT 0x51 acceleration frames and 0x53 angle frames.
 */

#ifndef __JY61P_H__
#define __JY61P_H__

#include <stdbool.h>
#include <stdint.h>

void JY61P_Init(void);
void JY61P_UART_IRQHandler(void);

/* Angle, degrees. */
float JY61P_GetYaw(void);
float JY61P_GetRoll(void);
float JY61P_GetPitch(void);
int16_t JY61P_GetYawRaw(void);
void JY61P_ZeroYaw(void);

/* WIT 0x51 acceleration, signed int16 over +/-16g. */
int16_t JY61P_GetAccelXRaw(void);
int16_t JY61P_GetAccelYRaw(void);
int16_t JY61P_GetAccelZRaw(void);
float JY61P_GetAccelXG(void);
float JY61P_GetAccelYG(void);
float JY61P_GetAccelZG(void);
float JY61P_GetAccelXMps2(void);
float JY61P_GetAccelYMps2(void);
float JY61P_GetAccelZMps2(void);

uint32_t JY61P_GetFrameCount(void);
uint32_t JY61P_GetAccelFrameCount(void);
bool JY61P_HasAcceleration(void);
bool JY61P_IsAccelFresh(void);

bool JY61P_IsOnline(void);
void JY61P_UpdateTick(void);

#endif /* __JY61P_H__ */
