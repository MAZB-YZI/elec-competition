#include "mpu6050.h"
#include "delay.h"

#include <stddef.h>

#define BNO055_REG_CHIP_ID       0x00U
#define BNO055_REG_EULER_H_LSB   0x1AU
#define BNO055_REG_CALIB_STAT    0x35U
#define BNO055_REG_UNIT_SEL      0x3BU
#define BNO055_REG_OPR_MODE      0x3DU
#define BNO055_REG_PWR_MODE      0x3EU
#define BNO055_REG_SYS_TRIGGER   0x3FU
#define BNO055_REG_PAGE_ID       0x07U

#define BNO055_MODE_CONFIG       0x00U
#define BNO055_MODE_NDOF         0x0CU
#define BNO055_PWR_NORMAL        0x00U
#define BNO055_EULER_LSB_PER_DEG 16.0f

static uint8_t bno_addr = MPU6050_I2C_ADDRESS;
static uint8_t chip_id;
static uint32_t read_error_count;
static float heading_deg;
static float heading_prev;
static float yaw_deg;
static float yaw_offset;
static float roll_deg;
static float pitch_deg;
static float gyro_z_dps;
static uint8_t cal_sys;
static uint8_t cal_gyro;
static uint8_t cal_accel;
static uint8_t cal_mag;
static bool heading_valid;

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = { reg, value };
    bool ok = MPU6050_PlatformWrite(bno_addr, packet, sizeof(packet),
        MPU6050_I2C_TIMEOUT_TICKS);
    if (!ok) read_error_count++;
    delay_ms(2U);
    return ok;
}

static bool read_reg(uint8_t reg, uint8_t *data, size_t length)
{
    bool ok = MPU6050_PlatformWriteRead(bno_addr, &reg, 1U, data, length,
        MPU6050_I2C_TIMEOUT_TICKS);
    if (!ok) read_error_count++;
    return ok;
}

static int16_t make_i16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static float wrap_delta_180(float delta)
{
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static bool read_calib(void)
{
    uint8_t v;
    if (!read_reg(BNO055_REG_CALIB_STAT, &v, 1U)) return false;
    cal_sys = (v >> 6) & 0x03U;
    cal_gyro = (v >> 4) & 0x03U;
    cal_accel = (v >> 2) & 0x03U;
    cal_mag = v & 0x03U;
    return true;
}

bool MPU6050_Init(void)
{
    const uint8_t addrs[2] = { 0x29U, 0x28U };
    uint8_t id = 0U;

    read_error_count = 0U;
    chip_id = 0U;
    heading_valid = false;
    yaw_deg = 0.0f;
    yaw_offset = 0.0f;

    delay_ms(700U);

    for (uint8_t a = 0U; a < 2U; ++a) {
        bno_addr = addrs[a];
        for (uint8_t i = 0U; i < 20U; ++i) {
            if (read_reg(BNO055_REG_CHIP_ID, &id, 1U)) {
                chip_id = id;
                if (id == MPU6050_WHO_AM_I_EXPECTED) break;
            }
            delay_ms(50U);
        }
        if (chip_id == MPU6050_WHO_AM_I_EXPECTED) break;
    }

    if (chip_id != MPU6050_WHO_AM_I_EXPECTED) return false;

    if (!write_reg(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG)) return false;
    delay_ms(30U);
    if (!write_reg(BNO055_REG_PAGE_ID, 0x00U)) return false;
    if (!write_reg(BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL)) return false;
    delay_ms(10U);
    if (!write_reg(BNO055_REG_UNIT_SEL, 0x00U)) return false; /* deg, C, m/s^2 */
    if (!write_reg(BNO055_REG_SYS_TRIGGER, 0x40U)) return false; /* external crystal if present */
    delay_ms(10U);
    if (!write_reg(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF)) return false;
    delay_ms(50U);

    (void)MPU6050_UpdateYawOnly(0.0f);
    MPU6050_ResetYaw();
    return true;
}

bool MPU6050_UpdateYawOnly(float dt_s)
{
    uint8_t b[6];
    float new_heading;
    float delta;
    (void)dt_s;

    if (!read_reg(BNO055_REG_EULER_H_LSB, b, sizeof(b))) return false;

    new_heading = (float)make_i16(b[0], b[1]) / BNO055_EULER_LSB_PER_DEG;
    roll_deg = (float)make_i16(b[2], b[3]) / BNO055_EULER_LSB_PER_DEG;
    pitch_deg = (float)make_i16(b[4], b[5]) / BNO055_EULER_LSB_PER_DEG;
    heading_deg = new_heading;

    if (!heading_valid) {
        heading_prev = new_heading;
        yaw_deg = wrap_delta_180(new_heading - yaw_offset);
        heading_valid = true;
    } else {
        delta = wrap_delta_180(new_heading - heading_prev);
        heading_prev = new_heading;
        yaw_deg += delta;
    }

    gyro_z_dps = 0.0f;
    (void)read_calib();
    return true;
}

bool MPU6050_Update(float dt_s) { return MPU6050_UpdateYawOnly(dt_s); }
bool MPU6050_ReadRaw(MPU6050Raw *raw) { if (raw) { raw->accel_x=raw->accel_y=raw->accel_z=raw->temp=raw->gyro_x=raw->gyro_y=raw->gyro_z=0; } return true; }
bool MPU6050_CalibrateGyro(uint16_t samples) { (void)samples; MPU6050_ResetYaw(); return true; }
float MPU6050_GetYaw(void) { return yaw_deg; }
float MPU6050_GetRoll(void) { return roll_deg; }
float MPU6050_GetPitch(void) { return pitch_deg; }
float MPU6050_GetGyroX(void) { return 0.0f; }
float MPU6050_GetGyroY(void) { return 0.0f; }
float MPU6050_GetGyroZ(void) { return gyro_z_dps; }
float MPU6050_GetGyroZBias(void) { return 0.0f; }
uint8_t MPU6050_GetWhoAmI(void) { return chip_id; }
uint8_t MPU6050_GetI2CAddress(void) { return bno_addr; }
uint32_t MPU6050_GetReadErrorCount(void) { return read_error_count; }
uint8_t BNO055_GetCalibSys(void) { return cal_sys; }
uint8_t BNO055_GetCalibGyro(void) { return cal_gyro; }
uint8_t BNO055_GetCalibAccel(void) { return cal_accel; }
uint8_t BNO055_GetCalibMag(void) { return cal_mag; }
float BNO055_GetHeading(void) { return heading_deg; }

void MPU6050_GetAttitude(MPU6050Attitude *attitude)
{
    if (!attitude) return;
    attitude->roll = roll_deg;
    attitude->pitch = pitch_deg;
    attitude->yaw = yaw_deg;
    attitude->gyro_x = 0.0f;
    attitude->gyro_y = 0.0f;
    attitude->gyro_z = gyro_z_dps;
}

void MPU6050_ResetYaw(void)
{
    yaw_offset = heading_deg;
    yaw_deg = 0.0f;
    heading_prev = heading_deg;
    heading_valid = true;
}

void MPU6050_ResetYawTo(float yaw)
{
    MPU6050_ResetYaw();
    yaw_deg = yaw;
}
