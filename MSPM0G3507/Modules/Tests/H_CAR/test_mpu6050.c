#include "mpu6050.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static uint8_t registers[128];
static uint8_t last_written_register;

static void set_i16(uint8_t reg, int16_t value)
{
    registers[reg] = (uint8_t) (((uint16_t) value >> 8) & 0xFFU);
    registers[reg + 1U] = (uint8_t) ((uint16_t) value & 0xFFU);
}

bool MPU6050_PlatformWrite(uint8_t address, const uint8_t *data,
    size_t length, uint32_t timeout_ticks)
{
    (void) timeout_ticks;
    assert(address == MPU6050_I2C_ADDRESS);
    assert(data != 0);
    assert(length >= 2U);

    last_written_register = data[0];
    for (size_t i = 1U; i < length; ++i) {
        registers[data[0] + i - 1U] = data[i];
    }
    return true;
}

bool MPU6050_PlatformWriteRead(uint8_t address, const uint8_t *tx,
    size_t tx_length, uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    (void) timeout_ticks;
    assert(address == MPU6050_I2C_ADDRESS);
    assert(tx != 0);
    assert(tx_length == 1U);
    assert(rx != 0);

    for (size_t i = 0U; i < rx_length; ++i) {
        rx[i] = registers[tx[0] + i];
    }
    return true;
}

int main(void)
{
    MPU6050Raw raw;

    registers[0x75U] = 0x68U;
    set_i16(0x3BU, 100);
    set_i16(0x3DU, -200);
    set_i16(0x3FU, 16384);
    set_i16(0x41U, 0);
    set_i16(0x43U, 10);
    set_i16(0x45U, -20);
    set_i16(0x47U, 131);

    assert(MPU6050_Init());
    assert(last_written_register == 0x1CU);
    assert(registers[0x6BU] == 0x01U);
    assert(registers[0x1AU] == 0x03U);
    assert(registers[0x1BU] == 0x00U);
    assert(registers[0x1CU] == 0x00U);

    assert(MPU6050_ReadRaw(&raw));
    assert(raw.accel_x == 100);
    assert(raw.accel_y == -200);
    assert(raw.accel_z == 16384);
    assert(raw.gyro_z == 131);

    assert(MPU6050_CalibrateGyro(1U));
    set_i16(0x47U, 262);
    assert(MPU6050_Update(1.0f));
    assert(fabsf(MPU6050_GetYaw() - 1.0f) < 0.001f);

    MPU6050_ResetYaw();
    assert(fabsf(MPU6050_GetYaw()) < 0.001f);

    return 0;
}
