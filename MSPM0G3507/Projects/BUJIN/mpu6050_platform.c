/**
 * mpu6050_platform.c — 已停用，I2C 已从 SysConfig 移除
 *
 * 当前工程使用 JY61P (UART0) 代替 BNO055/MPU6050 (I2C)。
 * 此文件保留为空壳以避免链接错误。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data,
    size_t length, uint32_t timeout_ticks)
{
    (void)address; (void)data; (void)length; (void)timeout_ticks;
    return false;
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    (void)address; (void)tx; (void)tx_length;
    (void)rx; (void)rx_length; (void)timeout_ticks;
    return false;
}
