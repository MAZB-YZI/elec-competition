#include "mpu6050.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_i2c.h"

#include <stddef.h>

static bool wait_i2c_idle(uint32_t timeout_ticks)
{
    while (timeout_ticks-- > 0U) {
        if (DL_I2C_getControllerStatus(I2C_BUS_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) {
            return true;
        }
    }
    return false;
}

static bool wait_i2c_done(uint32_t timeout_ticks)
{
    while (timeout_ticks-- > 0U) {
        uint32_t status = DL_I2C_getControllerStatus(I2C_BUS_INST);
        if (status & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        if (status & DL_I2C_CONTROLLER_STATUS_IDLE) {
            return true;
        }
    }
    return false;
}

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data,
    size_t length, uint32_t timeout_ticks)
{
    uint16_t written;

    if ((data == NULL) || (length == 0U) || (length > 255U)) {
        return false;
    }

    if (!wait_i2c_idle(timeout_ticks)) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_BUS_INST);
    written = DL_I2C_fillControllerTXFIFO(I2C_BUS_INST, data,
        (uint16_t)length);
    if (written != (uint16_t)length) {
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_BUS_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint16_t)length);
    return wait_i2c_done(timeout_ticks * 20U);
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    if ((tx == NULL) || (tx_length == 0U) || (tx_length > 255U) ||
        (rx == NULL) || (rx_length == 0U) || (rx_length > 255U)) {
        return false;
    }

    if (!MPU6050_PlatformWrite(address, tx, tx_length, timeout_ticks)) {
        return false;
    }

    if (!wait_i2c_idle(timeout_ticks)) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_BUS_INST);
    DL_I2C_startControllerTransfer(I2C_BUS_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint16_t)rx_length);

    for (size_t i = 0U; i < rx_length; ++i) {
        uint32_t guard = timeout_ticks * 20U;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_BUS_INST)) {
            if (guard-- == 0U) {
                return false;
            }
        }
        rx[i] = DL_I2C_receiveControllerData(I2C_BUS_INST);
    }

    return wait_i2c_done(timeout_ticks * 20U);
}
