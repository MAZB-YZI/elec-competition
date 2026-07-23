#include "jy61p.h"

/*
 * MPU6050 bring-up build:
 * PA0/PA1 are reassigned from JY61P UART0 to software I2C, so the old JY61P
 * UART driver is intentionally stubbed out in this copied test project.
 */

void JY61P_Init(void) {}
void JY61P_UART_IRQHandler(void) {}
float JY61P_GetYaw(void) { return 0.0f; }
int16_t JY61P_GetYawRaw(void) { return 0; }
float JY61P_GetRoll(void) { return 0.0f; }
float JY61P_GetPitch(void) { return 0.0f; }
uint32_t JY61P_GetFrameCount(void) { return 0U; }
void JY61P_ZeroYaw(void) {}
bool JY61P_IsOnline(void) { return false; }
void JY61P_UpdateTick(void) {}
