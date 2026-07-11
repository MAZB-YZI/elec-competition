#include "mpu6050.h"

#define MPU6050_REG_SMPLRT_DIV 0x19U
#define MPU6050_REG_CONFIG 0x1AU
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1 0x6BU
#define MPU6050_REG_WHO_AM_I 0x75U
#define MPU6050_WHO_AM_I_VALUE 0x68U
#define MPU6050_GYRO_LSB_PER_DPS 131.0f

static float gyro_z_bias;
static float yaw_deg;

static int16_t make_i16(uint8_t high, uint8_t low)
{
    return (int16_t) (((uint16_t) high << 8) | low);
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t packet[2];

    packet[0] = reg;
    packet[1] = value;
    return MPU6050_PlatformWrite(MPU6050_I2C_ADDRESS, packet, sizeof(packet),
        MPU6050_I2C_TIMEOUT_TICKS);
}

static bool read_reg(uint8_t reg, uint8_t *data, size_t length)
{
    return MPU6050_PlatformWriteRead(MPU6050_I2C_ADDRESS, &reg, 1U, data,
        length, MPU6050_I2C_TIMEOUT_TICKS);
}

bool MPU6050_Init(void)
{
    uint8_t id = 0U;

    gyro_z_bias = 0.0f;
    yaw_deg = 0.0f;

    if (!read_reg(MPU6050_REG_WHO_AM_I, &id, 1U)) {
        return false;
    }

    if (id != MPU6050_WHO_AM_I_VALUE) {
        return false;
    }

    return write_reg(MPU6050_REG_PWR_MGMT_1, 0x01U) &&
           write_reg(MPU6050_REG_SMPLRT_DIV, 0x04U) &&
           write_reg(MPU6050_REG_CONFIG, 0x03U) &&
           write_reg(MPU6050_REG_GYRO_CONFIG, 0x00U) &&
           write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00U);
}

bool MPU6050_ReadRaw(MPU6050Raw *raw)
{
    uint8_t buffer[14];

    if (raw == 0) {
        return false;
    }

    if (!read_reg(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
        return false;
    }

    raw->accel_x = make_i16(buffer[0], buffer[1]);
    raw->accel_y = make_i16(buffer[2], buffer[3]);
    raw->accel_z = make_i16(buffer[4], buffer[5]);
    raw->temperature = make_i16(buffer[6], buffer[7]);
    raw->gyro_x = make_i16(buffer[8], buffer[9]);
    raw->gyro_y = make_i16(buffer[10], buffer[11]);
    raw->gyro_z = make_i16(buffer[12], buffer[13]);
    return true;
}

bool MPU6050_CalibrateGyro(uint16_t samples)
{
    MPU6050Raw raw;
    int32_t sum = 0;

    if (samples == 0U) {
        return false;
    }

    for (uint16_t i = 0; i < samples; ++i) {
        if (!MPU6050_ReadRaw(&raw)) {
            return false;
        }
        sum += raw.gyro_z;
    }

    gyro_z_bias = (float) sum / (float) samples;
    return true;
}

bool MPU6050_Update(float dt_s)
{
    MPU6050Raw raw;
    float gyro_z_dps;

    if (dt_s <= 0.0f) {
        return false;
    }

    if (!MPU6050_ReadRaw(&raw)) {
        return false;
    }

    gyro_z_dps = ((float) raw.gyro_z - gyro_z_bias) / MPU6050_GYRO_LSB_PER_DPS;
    yaw_deg += gyro_z_dps * dt_s;
    return true;
}

float MPU6050_GetYaw(void)
{
    return yaw_deg;
}

void MPU6050_ResetYaw(void)
{
    yaw_deg = 0.0f;
}
