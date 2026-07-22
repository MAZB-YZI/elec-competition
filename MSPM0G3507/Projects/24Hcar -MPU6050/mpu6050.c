#include "mpu6050.h"
#include "delay.h"
#include <math.h>

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_GYRO_ZOUT_H   0x47U
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_PWR_MGMT_2    0x6CU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_RAD_TO_DEG        57.2957795f
#define MPU6050_GYRO_LSB_PER_DPS  16.4f
#define MPU6050_ACCEL_LSB_PER_G   16384.0f
#define MPU6050_GYRO_Z_DEADBAND   0.20f
#define MPU6050_STILL_GYRO_DPS    1.2f
#define MPU6050_STILL_ACCEL_ERR_G 0.06f
#define MPU6050_BIAS_TRACK_ALPHA  0.0f
#define MPU6050_GYRO_Z_LPF_HZ     30.0f
#define MPU6050_MAX_DT_S          1.000f
#define MPU6050_CAL_VERIFY_SAMPLES 100U
#define MPU6050_CAL_MAX_RES_RAW   5.0f

typedef struct {
    float q_angle;
    float q_bias;
    float r_measure;
    float angle;
    float bias;
    float p[2][2];
} Kalman1D;

static float gyro_z_bias_raw;
static float gyro_x_dps;
static float gyro_y_dps;
static float gyro_z_dps;
static float gyro_z_lpf_dps;
static float roll_deg;
static float pitch_deg;
static float yaw_deg;
static uint8_t last_who_am_i;
static uint32_t read_error_count;
static Kalman1D kalman_roll;
static Kalman1D kalman_pitch;

static float pt1_apply(float prev, float input, float cutoff_hz, float dt_s)
{
    float rc;
    float alpha;

    if ((cutoff_hz <= 0.0f) || (dt_s <= 0.0f)) {
        return input;
    }

    rc = 1.0f / (2.0f * 3.1415926f * cutoff_hz);
    alpha = dt_s / (rc + dt_s);
    return prev + (alpha * (input - prev));
}

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

static void kalman_init(Kalman1D *kalman)
{
    kalman->q_angle = 0.001f;
    kalman->q_bias = 0.003f;
    kalman->r_measure = 0.03f;
    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    kalman->p[0][0] = 0.0f;
    kalman->p[0][1] = 0.0f;
    kalman->p[1][0] = 0.0f;
    kalman->p[1][1] = 0.0f;
}

static float kalman_get_angle(Kalman1D *kalman, float new_angle,
    float new_rate, float dt_s)
{
    float rate;
    float s;
    float k0;
    float k1;
    float y;
    float p00_temp;
    float p01_temp;

    rate = new_rate - kalman->bias;
    kalman->angle += dt_s * rate;

    kalman->p[0][0] += dt_s * ((dt_s * kalman->p[1][1]) -
        kalman->p[0][1] - kalman->p[1][0] + kalman->q_angle);
    kalman->p[0][1] -= dt_s * kalman->p[1][1];
    kalman->p[1][0] -= dt_s * kalman->p[1][1];
    kalman->p[1][1] += kalman->q_bias * dt_s;

    s = kalman->p[0][0] + kalman->r_measure;
    if (s == 0.0f) {
        return kalman->angle;
    }

    k0 = kalman->p[0][0] / s;
    k1 = kalman->p[1][0] / s;
    y = new_angle - kalman->angle;
    kalman->angle += k0 * y;
    kalman->bias += k1 * y;

    p00_temp = kalman->p[0][0];
    p01_temp = kalman->p[0][1];
    kalman->p[0][0] -= k0 * p00_temp;
    kalman->p[0][1] -= k0 * p01_temp;
    kalman->p[1][0] -= k1 * p00_temp;
    kalman->p[1][1] -= k1 * p01_temp;

    return kalman->angle;
}

bool MPU6050_Init(void)
{
    uint8_t id = 0U;
    bool id_ok = false;

    gyro_z_bias_raw = 0.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;
    gyro_z_lpf_dps = 0.0f;
    roll_deg = 0.0f;
    pitch_deg = 0.0f;
    yaw_deg = 0.0f;
    last_who_am_i = 0U;
    read_error_count = 0U;
    kalman_init(&kalman_roll);
    kalman_init(&kalman_pitch);

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
    int32_t verify_sum_z = 0;
    int32_t sum_ax = 0;
    int32_t sum_ay = 0;
    int32_t sum_az = 0;

    if (samples == 0U) {
        return false;
    }

    for (uint16_t i = 0U; i < samples; ++i) {
        if (!MPU6050_ReadRaw(&raw)) {
            return false;
        }
        sum_z += raw.gyro_z;
        sum_ax += raw.accel_x;
        sum_ay += raw.accel_y;
        sum_az += raw.accel_z;
        delay_ms(1U);
    }

    gyro_z_bias_raw = (float) sum_z / (float) samples;
    gyro_z_dps = 0.0f;
    gyro_z_lpf_dps = 0.0f;
    yaw_deg = 0.0f;

    {
        float ax = (float) sum_ax / (float) samples;
        float ay = (float) sum_ay / (float) samples;
        float az = (float) sum_az / (float) samples;
        float roll_sqrt = sqrtf((ax * ax) + (az * az));

        if (roll_sqrt != 0.0f) {
            roll_deg = atanf(ay / roll_sqrt) * MPU6050_RAD_TO_DEG;
        } else {
            roll_deg = 0.0f;
        }
        pitch_deg = atan2f(-ax, az) * MPU6050_RAD_TO_DEG;

        kalman_init(&kalman_roll);
        kalman_init(&kalman_pitch);
        kalman_roll.angle = roll_deg;
        kalman_pitch.angle = pitch_deg;
    }

    for (uint16_t i = 0U; i < MPU6050_CAL_VERIFY_SAMPLES; ++i) {
        if (!MPU6050_ReadRaw(&raw)) {
            return false;
        }
        verify_sum_z += raw.gyro_z;
        delay_ms(1U);
    }

    if (fabsf(((float) verify_sum_z / (float) MPU6050_CAL_VERIFY_SAMPLES) -
        gyro_z_bias_raw) > MPU6050_CAL_MAX_RES_RAW) {
        return false;
    }

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
    float gyro_x_raw_dps;
    float gyro_y_raw_dps;
    float accel_roll_deg;
    float accel_pitch_deg;
    float roll_sqrt;

    if (dt_s <= 0.0f) {
        return false;
    }
    if (dt_s > MPU6050_MAX_DT_S) {
        dt_s = MPU6050_MAX_DT_S;
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
    gyro_x_raw_dps = (float) raw.gyro_x / MPU6050_GYRO_LSB_PER_DPS;
    gyro_y_raw_dps = (float) raw.gyro_y / MPU6050_GYRO_LSB_PER_DPS;
    gyro_x_dps = gyro_x_raw_dps;
    gyro_y_dps = gyro_y_raw_dps;

    roll_sqrt = sqrtf(((float) raw.accel_x * (float) raw.accel_x) +
                      ((float) raw.accel_z * (float) raw.accel_z));
    if (roll_sqrt != 0.0f) {
        accel_roll_deg = atanf((float) raw.accel_y / roll_sqrt) *
            MPU6050_RAD_TO_DEG;
    } else {
        accel_roll_deg = 0.0f;
    }
    accel_pitch_deg = atan2f((float) -raw.accel_x, (float) raw.accel_z) *
        MPU6050_RAD_TO_DEG;

    if (((accel_pitch_deg < -90.0f) && (pitch_deg > 90.0f)) ||
        ((accel_pitch_deg > 90.0f) && (pitch_deg < -90.0f))) {
        kalman_pitch.angle = accel_pitch_deg;
        pitch_deg = accel_pitch_deg;
    } else {
        pitch_deg = kalman_get_angle(&kalman_pitch, accel_pitch_deg,
        gyro_y_raw_dps, dt_s);
    }

    if ((pitch_deg > 90.0f) || (pitch_deg < -90.0f)) {
        gyro_x_raw_dps = -gyro_x_raw_dps;
    }
    roll_deg = kalman_get_angle(&kalman_roll, accel_roll_deg,
        gyro_x_raw_dps, dt_s);

    /*
     * When the board is nearly still and the accel magnitude is close to 1 g,
     * slowly track the gyro zero bias. This reduces long-term yaw drift while
     * avoiding aggressive bias changes during real turns.
     */
    if ((MPU6050_BIAS_TRACK_ALPHA > 0.0f) &&
        (gyro_z_unbiased_dps > -MPU6050_STILL_GYRO_DPS) &&
        (gyro_z_unbiased_dps < MPU6050_STILL_GYRO_DPS) &&
        (accel_mag_g > (1.0f - MPU6050_STILL_ACCEL_ERR_G)) &&
        (accel_mag_g < (1.0f + MPU6050_STILL_ACCEL_ERR_G))) {
        gyro_z_bias_raw =
            (gyro_z_bias_raw * (1.0f - MPU6050_BIAS_TRACK_ALPHA)) +
            ((float) raw.gyro_z * MPU6050_BIAS_TRACK_ALPHA);
        gyro_z_unbiased_dps =
            ((float) raw.gyro_z - gyro_z_bias_raw) / MPU6050_GYRO_LSB_PER_DPS;
    }

    gyro_z_lpf_dps = pt1_apply(gyro_z_lpf_dps, gyro_z_unbiased_dps,
        MPU6050_GYRO_Z_LPF_HZ, dt_s);
    gyro_z_dps = gyro_z_lpf_dps;

    if ((gyro_z_dps > -MPU6050_GYRO_Z_DEADBAND) &&
        (gyro_z_dps < MPU6050_GYRO_Z_DEADBAND)) {
        gyro_z_dps = 0.0f;
    }

    yaw_deg = normalize_yaw(yaw_deg + (gyro_z_dps * dt_s));
    return true;
}

bool MPU6050_UpdateYawOnly(float dt_s)
{
    uint8_t buffer[2];
    int16_t gyro_z_raw;
    float gyro_z_unbiased_dps;

    if (dt_s <= 0.0f) {
        return false;
    }
    if (dt_s > MPU6050_MAX_DT_S) {
        dt_s = MPU6050_MAX_DT_S;
    }

    if (!read_reg(MPU6050_REG_GYRO_ZOUT_H, buffer, sizeof(buffer))) {
        ++read_error_count;
        return false;
    }

    gyro_z_raw = make_i16(buffer[0], buffer[1]);
    gyro_z_unbiased_dps =
        ((float)gyro_z_raw - gyro_z_bias_raw) / MPU6050_GYRO_LSB_PER_DPS;

    gyro_z_lpf_dps = pt1_apply(gyro_z_lpf_dps, gyro_z_unbiased_dps,
        MPU6050_GYRO_Z_LPF_HZ, dt_s);
    gyro_z_dps = gyro_z_lpf_dps;

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

float MPU6050_GetRoll(void)
{
    return roll_deg;
}

float MPU6050_GetPitch(void)
{
    return pitch_deg;
}

float MPU6050_GetGyroX(void)
{
    return gyro_x_dps;
}

float MPU6050_GetGyroY(void)
{
    return gyro_y_dps;
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

void MPU6050_GetAttitude(MPU6050Attitude *attitude)
{
    if (attitude == NULL) {
        return;
    }

    attitude->roll_deg = roll_deg;
    attitude->pitch_deg = pitch_deg;
    attitude->yaw_deg = yaw_deg;
    attitude->gyro_x_dps = gyro_x_dps;
    attitude->gyro_y_dps = gyro_y_dps;
    attitude->gyro_z_dps = gyro_z_dps;
    attitude->gyro_z_bias_raw = gyro_z_bias_raw;
}

void MPU6050_ResetYaw(void)
{
    yaw_deg = 0.0f;
    gyro_z_lpf_dps = 0.0f;
}

void MPU6050_ResetYawTo(float yaw)
{
    yaw_deg = normalize_yaw(yaw);
    gyro_z_lpf_dps = 0.0f;
}
