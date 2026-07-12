#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Software I2C driver for the H_CAR MPU6050 connector.
 *
 * The Tianmengxing extension board wires MPU6050 SDA/SCL to PA0/PA1. Those
 * pins cannot be used together with the OLED hardware-I2C pins PA28/PA31 as a
 * second I2C0 pin group, so this module bit-bangs PA0/PA1 as GPIO.
 */

void SoftI2C_Init(void);

void SoftI2C_TestReleaseBoth(void);
void SoftI2C_TestPullSdaLow(void);
void SoftI2C_TestPullSclLow(void);
bool SoftI2C_TestReadSda(void);
bool SoftI2C_TestReadScl(void);

bool SoftI2C_ProbeAddress(uint8_t addr);

bool SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);

bool SoftI2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data);

bool SoftI2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* SOFT_I2C_H */
