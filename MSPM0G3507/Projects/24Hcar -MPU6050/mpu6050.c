#include "mpu6050.h"
#include "delay.h"
#include <math.h>

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_PWR_MGMT_2    0x6CU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_GYRO_LSB_PER_DPS  16.4f
#define MPU6050_ACCEL_LSB_PER_G   16384.0f
#define MPU6050_GYRO_Z_DEADBAND   0.30f
#define MPU6050_STILL_GYRO_DPS    2.0f
#define MPU6050_STILL_ACCEL_ERR_G 0.08f
#define MPU6050_BIAS_TRACK_ALPHA  0.01f

static float gyro_z_bias_raw;
static float gyro_z_dps;
static float yaw_deg;
static uint8_t last_who_am_i;
static uint32_t read_error_count;

static int16_t make_i16(uint8_t high, uint8_t low)
{
    return (int16_t) (((uint16_t) high << 8) | low);
}

static float normalize_yaw(float yaw)
{
    while (yaw > 180.0f) {
        yaw -= 360.0f;
    }

    while (yaw < -180.0f) {
        yaw += 360.0f;
    }

    return yaw;
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
    if ((data == NULL) || (length == 0U)) {
        return false;
    }

    return MPU6050_PlatformWriteRead(MPU6050_I2C_ADDRESS, &reg, 1U, data,
        length, MPU6050_I2C_TIMEOUT_TICKS);
}

bool MPU6050_Init(void)
{
    uint8_t id = 0U;
    bool id_ok = false;

    gyro_z_bias_raw = 0.0f;
    gyro_z_dps = 0.0f;
    yaw_deg = 0.0f;
    last_who_am_i = 0U;
    read_error_count = 0U;

    for (uint8_t attempt = 0U; attempt < 5U; ++attempt) {
        if (read_reg(MPU6050_REG_WHO_AM_I, &id, 1U)) {
            id_ok = true;
            break;
        }
        ++read_error_count;
    }

    if (!id_ok) {
        return false;
    }

    last_who_am_i = id;
    if (id != MPU6050_WHO_AM_I_EXPECTED) {
        return false;
    }

    /*
     * Minimal safe raw-gyro setup:
     * - wake the chip and use X gyro PLL clock
     * - enable all accel/gyro axes
     * - 1 kHz / (1 + 4) = 200 Hz raw sample rate
     * - DLPF_CFG=3, about 44 Hz gyro bandwidth
     * - gyro ±2000 dps, accel ±2 g
     *
     * Fast hand turns and in-place car turns can exceed ±250 dps.  Use a
     * widest gyro range so quick 90-degree tests do not saturate the sensor.
     */
    return write_reg(MPU6050_REG_PWR_MGMT_1, 0x01U) &&
           write_reg(MPU6050_REG_PWR_MGMT_2, 0x00U) &&
           write_reg(MPU6050_REG_SMPLRT_DIV, 0x04U) &&
           write_reg(MPU6050_REG_CONFIG, 0x03U) &&
           write_reg(MPU6050_REG_GYRO_CONFIG, 0x18U) &&
           write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00U);
}

bool MPU6050_ReadRaw(MPU6050Raw *raw)
{
    uint8_t buffer[14];

    if (raw == NULL) {
        return false;
    }

    if (!read_reg(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
        ++read_error_count;
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
    int32_t sum_z = 0;

    if (samples == 0U) {
        return false;
    }

    for (uint16_t i = 0U; i < samples; ++i) {
        if (!MPU6050_ReadRaw(&raw)) {
            return false;
        }
        sum_z += raw.gyro_z;
        delay_ms(2U);
    }

    gyro_z_bias_raw = (float) sum_z / (float) samples;
    gyro_z_dps = 0.0f;
    yaw_deg = 0.0f;
    return true;
}

bool MPU6050_Update(float dt_s)
{
    MPU6050Raw raw;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float accel_mag_g;
    float gyro_z_unbiased_dps;

    if (dt_s <= 0.0f) {
        return false;
    }

    if (!MPU6050_ReadRaw(&raw)) {
        return false;
    }

    gyro_z_unbiased_dps =
        ((float) raw.gyro_z - gyro_z_bias_raw) / MPU6050_GYRO_LSB_PER_DPS;

    accel_x_g = (float) raw.accel_x / MPU6050_ACCEL_LSB_PER_G;
    accel_y_g = (float) raw.accel_y / MPU6050_ACCEL_LSB_PER_G;
    accel_z_g = (float) raw.accel_z / MPU6050_ACCEL_LSB_PER_G;
    accel_mag_g = sqrtf((accel_x_g * accel_x_g) + (accel_y_g * accel_y_g) +
                        (accel_z_g * accel_z_g));

    /*
     * When the board is nearly still and the accel magnitude is close to 1 g,
     * slowly track the gyro zero bias. This reduces long-term yaw drift while
     * avoiding aggressive bias changes during real turns.
     */
    if ((gyro_z_unbiased_dps > -MPU6050_STILL_GYRO_DPS) &&
        (gyro_z_unbiased_dps < MPU6050_STILL_GYRO_DPS) &&
        (accel_mag_g > (1.0f - MPU6050_STILL_ACCEL_ERR_G)) &&
        (accel_mag_g < (1.0f + MPU6050_STILL_ACCEL_ERR_G))) {
        gyro_z_bias_raw =
            (gyro_z_bias_raw * (1.0f - MPU6050_BIAS_TRACK_ALPHA)) +
            ((float) raw.gyro_z * MPU6050_BIAS_TRACK_ALPHA);
        gyro_z_unbiased_dps =
            ((float) raw.gyro_z - gyro_z_bias_raw) / MPU6050_GYRO_LSB_PER_DPS;
    }

    gyro_z_dps = gyro_z_unbiased_dps;

    if ((gyro_z_dps > -MPU6050_GYRO_Z_DEADBAND) &&
        (gyro_z_dps < MPU6050_GYRO_Z_DEADBAND)) {
        gyro_z_dps = 0.0f;
    }

    yaw_deg = normalize_yaw(yaw_deg + (gyro_z_dps * dt_s));
    return true;
}

float MPU6050_GetYaw(void)
{
    return yaw_deg;
}

float MPU6050_GetGyroZ(void)
{
    return gyro_z_dps;
}

float MPU6050_GetGyroZBias(void)
{
    return gyro_z_bias_raw;
}

uint8_t MPU6050_GetWhoAmI(void)
{
    return last_who_am_i;
}

uint32_t MPU6050_GetReadErrorCount(void)
{
    return read_error_count;
}

void MPU6050_ResetYaw(void)
{
    yaw_deg = 0.0f;
}

void MPU6050_ResetYawTo(float yaw)
{
    yaw_deg = normalize_yaw(yaw);
}
