#ifndef HCAR_HAL_H
#define HCAR_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 1ms tick callback - called from timer interrupt */
void HCar_1msTickCallback(void);

/* Implement these hooks with SysConfig-generated names after the CCS project
 * and pin configuration have been created with the CCS/SysConfig tools. */
bool HCarHal_Init(void);
bool HCarHal_Start1msTick(void);
void HCarHal_Idle(void);
void HCarHal_SetMotorDirection(bool left, bool in1, bool in2);
void HCarHal_SetMotorDuty(bool left, uint16_t duty);
uint16_t HCarHal_GetMotorPeriod(void);
bool HCarHal_ReadEncoderB(bool left);
void HCarHal_SetBuzzer(bool on);
/* Tianmengxing onboard status LED: PB22. HAL handles active polarity. */
void HCarHal_SetStatusLed(bool on);
bool HCarHal_I2CWrite(uint8_t address, const uint8_t *data, size_t length,
    uint32_t timeout_ticks);
bool HCarHal_I2CWriteRead(uint8_t address, const uint8_t *tx, size_t tx_length,
    uint8_t *rx, size_t rx_length, uint32_t timeout_ticks);

#endif
