#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 软件 I2C 驱动
 * 使用 GPIO 模拟 I2C 时序
 * 适用于 PA0 (SDA) / PA1 (SCL)
 */

/* 初始化软件 I2C 引脚 */
void SoftI2C_Init(void);

/* 写一个字节到指定寄存器 */
bool SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);

/* 从指定寄存器读一个字节 */
bool SoftI2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data);

/* 从指定寄存器读多个字节 */
bool SoftI2C_ReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* SOFT_I2C_H */
