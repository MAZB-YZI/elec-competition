#include "mpu6050.h"
#include "soft_i2c.h"

#include <stddef.h>

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data,
    size_t length, uint32_t timeout_ticks)
{
    (void) timeout_ticks;

    if ((data == NULL) || (length < 2U)) {
        return false;
    }

    /*
     * The generic MPU6050 layer passes [register, data0, data1...].  The
     * bring-up driver only needs the one-byte register writes used by init.
     */
    uint8_t reg = data[0];
    for (size_t i = 1U; i < length; ++i) {
        if (!SoftI2C_WriteReg(address, reg, data[i])) {
            return false;
        }
        ++reg;
    }

    return true;
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    (void) timeout_ticks;

    if ((tx == NULL) || (tx_length != 1U) || (rx == NULL) || (rx_length == 0U)) {
        return false;
    }

    return SoftI2C_ReadBytes(address, tx[0], rx, (uint8_t) rx_length);
}
