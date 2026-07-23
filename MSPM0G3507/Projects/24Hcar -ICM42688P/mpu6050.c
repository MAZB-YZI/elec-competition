#include "mpu6050.h"
#include "delay.h"

#include <math.h>
#include <stddef.h>

#define ICM42688_REG_DEVICE_CONFIG 0x11U
#define ICM42688_REG_ACCEL_X1      0x1FU
#define ICM42688_REG_GYRO_X1       0x25U
#define ICM42688_REG_GYRO_Z1       0x29U
#define ICM42688_REG_PWR_MGMT0     0x4EU
#define ICM42688_REG_GYRO_CONFIG0  0x4FU
#define ICM42688_REG_ACCEL_CONFIG0 0x50U
#define ICM42688_REG_WHO_AM_I      0x75U
#define ICM42688_REG_BANK_SEL      0x76U

#define ICM42688_GYRO_LSB_PER_DPS  32.768f  /* +/-1000 dps */
#define ICM42688_ACCEL_LSB_PER_G   8192.0f  /* +/-4g */
#define ICM42688_RAD_TO_DEG        57.2957795f
#define ICM42688_GYRO_Z_DEADBAND   0.08f
#define ICM42688_GYRO_Z_LPF_HZ     80.0f
#define ICM42688_MAX_DT_S          1.000f

static uint8_t who_am_i;
static uint8_t icm_i2c_address = MPU6050_I2C_ADDRESS;
static uint32_t read_error_count;
static float gyro_x_dps;
static float gyro_y_dps;
static float gyro_z_dps;
static float gyro_z_bias_raw;
static float accel_x_g;
static float accel_y_g;
static float accel_z_g;
static float roll_deg;
static float pitch_deg;
static float yaw_deg;
static bool gyro_z_lpf_ready;

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = { reg, value };
    bool ok = MPU6050_PlatformWrite(icm_i2c_address, packet,
        sizeof(packet), MPU6050_I2C_TIMEOUT_TICKS);
    if (!ok) {
        read_error_count++;
    }
    return ok;
}

static bool read_reg(uint8_t reg, uint8_t *data, size_t length)
{
    bool ok = MPU6050_PlatformWriteRead(icm_i2c_address, &reg, 1U, data,
        length, MPU6050_I2C_TIMEOUT_TICKS);
    if (!ok) {
        read_error_count++;
    }
    return ok;
}

static int16_t make_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static float low_pass(float prev, float input, float cutoff_hz, float dt_s)
{
    float rc = 1.0f / (6.2831853f * cutoff_hz);
    float alpha = dt_s / (rc + dt_s);
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    return prev + alpha * (input - prev);
}

bool MPU6050_Init(void)
{
    uint8_t id = 0U;

    who_am_i = 0U;
    read_error_count = 0U;

    const uint8_t addrs[2] = { 0x68U, 0x69U };
    for (uint8_t a = 0U; a < 2U; ++a) {
        icm_i2c_address = addrs[a];
        (void)write_reg(ICM42688_REG_BANK_SEL, 0x00U);
        delay_ms(10U);

        for (uint8_t i = 0U; i < 8U; ++i) {
            if (read_reg(ICM42688_REG_WHO_AM_I, &id, 1U)) {
                who_am_i = id;
                if (id == MPU6050_WHO_AM_I_EXPECTED) {
                    break;
                }
            }
            delay_ms(20U);
        }

        if (who_am_i == MPU6050_WHO_AM_I_EXPECTED) {
            break;
        }
    }

    if (who_am_i != MPU6050_WHO_AM_I_EXPECTED) {
        return false;
    }

    if (!write_reg(ICM42688_REG_DEVICE_CONFIG, 0x01U)) {
        return false;
    }
    delay_ms(100U);

    return write_reg(ICM42688_REG_BANK_SEL, 0x00U) &&
           write_reg(ICM42688_REG_GYRO_CONFIG0, 0x2FU) &&   /* +/-1000dps, 500Hz */
           write_reg(ICM42688_REG_ACCEL_CONFIG0, 0x4FU) &&  /* +/-4g, 500Hz */
           write_reg(ICM42688_REG_PWR_MGMT0, 0x0FU);
}

bool MPU6050_ReadRaw(MPU6050Raw *raw)
{
    uint8_t buffer[12];

    if (raw == NULL) {
        return false;
    }

    if (!read_reg(ICM42688_REG_ACCEL_X1, buffer, sizeof(buffer))) {
        return false;
    }

    raw->accel_x = make_i16(buffer[0], buffer[1]);
    raw->accel_y = make_i16(buffer[2], buffer[3]);
    raw->accel_z = make_i16(buffer[4], buffer[5]);
    raw->temp = 0;
    raw->gyro_x = make_i16(buffer[6], buffer[7]);
    raw->gyro_y = make_i16(buffer[8], buffer[9]);
    raw->gyro_z = make_i16(buffer[10], buffer[11]);
    return true;
}

bool MPU6050_CalibrateGyro(uint16_t samples)
{
    int64_t sum_z = 0;
    uint8_t buffer[2];

    if (samples == 0U) {
        return false;
    }

    gyro_z_bias_raw = 0.0f;
    gyro_z_lpf_ready = false;

    for (uint16_t i = 0U; i < samples; ++i) {
        if (!read_reg(ICM42688_REG_GYRO_Z1, buffer, sizeof(buffer))) {
            return false;
        }
        sum_z += make_i16(buffer[0], buffer[1]);
        delay_ms(1U);
    }

    gyro_z_bias_raw = (float)sum_z / (float)samples;
    MPU6050_ResetYaw();
    return true;
}

bool MPU6050_UpdateYawOnly(float dt_s)
{
    uint8_t buffer[2];
    int16_t gz_raw;
    float gz_unbiased_dps;

    if (dt_s <= 0.0f) {
        return true;
    }
    if (dt_s > ICM42688_MAX_DT_S) {
        dt_s = ICM42688_MAX_DT_S;
    }

    if (!read_reg(ICM42688_REG_GYRO_Z1, buffer, sizeof(buffer))) {
        return false;
    }

    gz_raw = make_i16(buffer[0], buffer[1]);
    gz_unbiased_dps = ((float)gz_raw - gyro_z_bias_raw) / ICM42688_GYRO_LSB_PER_DPS;

    if (!gyro_z_lpf_ready) {
        gyro_z_dps = gz_unbiased_dps;
        gyro_z_lpf_ready = true;
    } else {
        gyro_z_dps = low_pass(gyro_z_dps, gz_unbiased_dps, ICM42688_GYRO_Z_LPF_HZ, dt_s);
    }

    if ((gyro_z_dps > -ICM42688_GYRO_Z_DEADBAND) &&
        (gyro_z_dps < ICM42688_GYRO_Z_DEADBAND)) {
        gyro_z_dps = 0.0f;
    }

    yaw_deg += gyro_z_dps * dt_s;
    return true;
}

bool MPU6050_Update(float dt_s)
{
    MPU6050Raw raw;
    float ax, ay, az;

    if (!MPU6050_ReadRaw(&raw)) {
        return false;
    }

    ax = (float)raw.accel_x / ICM42688_ACCEL_LSB_PER_G;
    ay = (float)raw.accel_y / ICM42688_ACCEL_LSB_PER_G;
    az = (float)raw.accel_z / ICM42688_ACCEL_LSB_PER_G;
    accel_x_g = ax;
    accel_y_g = ay;
    accel_z_g = az;

    gyro_x_dps = (float)raw.gyro_x / ICM42688_GYRO_LSB_PER_DPS;
    gyro_y_dps = (float)raw.gyro_y / ICM42688_GYRO_LSB_PER_DPS;

    roll_deg = atan2f(ay, az) * ICM42688_RAD_TO_DEG;
    pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * ICM42688_RAD_TO_DEG;
    return MPU6050_UpdateYawOnly(dt_s);
}

float MPU6050_GetYaw(void) { return yaw_deg; }
float MPU6050_GetRoll(void) { return roll_deg; }
float MPU6050_GetPitch(void) { return pitch_deg; }
float MPU6050_GetGyroX(void) { return gyro_x_dps; }
float MPU6050_GetGyroY(void) { return gyro_y_dps; }
float MPU6050_GetGyroZ(void) { return gyro_z_dps; }
float MPU6050_GetGyroZBias(void) { return gyro_z_bias_raw; }
uint8_t MPU6050_GetWhoAmI(void) { return who_am_i; }
uint8_t MPU6050_GetI2CAddress(void) { return icm_i2c_address; }
uint32_t MPU6050_GetReadErrorCount(void) { return read_error_count; }

void MPU6050_GetAttitude(MPU6050Attitude *attitude)
{
    if (attitude == NULL) {
        return;
    }
    attitude->roll = roll_deg;
    attitude->pitch = pitch_deg;
    attitude->yaw = yaw_deg;
    attitude->gyro_x = gyro_x_dps;
    attitude->gyro_y = gyro_y_dps;
    attitude->gyro_z = gyro_z_dps;
}

void MPU6050_ResetYaw(void)
{
    yaw_deg = 0.0f;
    gyro_z_dps = 0.0f;
    gyro_z_lpf_ready = false;
}

void MPU6050_ResetYawTo(float yaw)
{
    yaw_deg = yaw;
    gyro_z_dps = 0.0f;
    gyro_z_lpf_ready = false;
}


