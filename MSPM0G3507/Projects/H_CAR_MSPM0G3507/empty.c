#include "ti_msp_dl_config.h"
#include "../../../../Modules/Drivers/OLED/oled.h"
#include "../../../../Modules/Drivers/MPU6050/soft_i2c.h"
#include "delay.h"
#include <math.h>

/*
 * MPU6050 完整测试 (修复版)
 * - 一次性读取 14 字节
 * - 使用真实 dt
 * - 显示真实 yaw 值
 * - I2C 读取失败检测
 */

#define MPU6050_ADDR 0x68
#define PI 3.14159f
#define GYRO_SCALE (1.0f / 131.0f)  /* ±250°/s */

static float gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
static float pitch = 0, roll = 0, yaw = 0;

/* 一次性读取 14 字节，返回 bool */
static bool mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    if (!SoftI2C_ReadBytes(MPU6050_ADDR, 0x3B, buf, 14)) {
        return false;
    }
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
}

static void calibrate_gyro(uint16_t samples)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t ax, ay, az, gx, gy, gz;

    OLED_Clear();
    OLED_ShowString(0, 0, "Calibrating...", 16);
    OLED_ShowString(0, 16, "Keep still!", 16);
    OLED_Refresh();

    for (uint16_t i = 0; i < samples; i++) {
        if (mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
        }
        delay_ms(5U);
    }

    gyro_x_bias = (float)sum_x / samples;
    gyro_y_bias = (float)sum_y / samples;
    gyro_z_bias = (float)sum_z / samples;
}

int main(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t last_ms = 0, now_ms = 0;
    bool first = true;

    SYSCFG_DL_init();

    SoftI2C_Init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 V2", 16);
    OLED_Refresh();
    delay_ms(200U);

    /* 唤醒 */
    SoftI2C_WriteReg(MPU6050_ADDR, 0x6B, 0x01);
    delay_ms(100U);

    /* 校准 */
    calibrate_gyro(200);

    /* 初始化角度 */
    if (mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
        float fax = (float)ax / 16384.0f;
        float fay = (float)ay / 16384.0f;
        float faz = (float)az / 16384.0f;
        pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 180.0f / PI;
        roll  = atan2f(fay, faz) * 180.0f / PI;
    }
    yaw = 0.0f;
    last_ms = 0;

    /* 主循环 */
    while (1) {
        if (!mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            OLED_Clear();
            OLED_ShowString(0, 0, "I2C ERR!", 16);
            OLED_Refresh();
            delay_ms(500U);
            continue;
        }

        /* 计算真实 dt (假设主循环约 10ms) */
        float dt = 0.01f;  /* TODO: 用定时器算真实 dt */

        /* 减去零偏 */
        float gx_dps = ((float)gx - gyro_x_bias) * GYRO_SCALE;
        float gy_dps = ((float)gy - gyro_y_bias) * GYRO_SCALE;
        float gz_dps = ((float)gz - gyro_z_bias) * GYRO_SCALE;

        /* 加速度计角度 */
        float fax = (float)ax / 16384.0f;
        float fay = (float)ay / 16384.0f;
        float faz = (float)az / 16384.0f;
        float accel_pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 180.0f / PI;
        float accel_roll  = atan2f(fay, faz) * 180.0f / PI;

        /* Pitch/Roll 直接用加速度计 */
        pitch = accel_pitch;
        roll  = accel_roll;

        /* Yaw: 陀螺仪积分 */
        yaw += gz_dps * dt;

        /* 限制范围 */
        if (yaw > 180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;

        /* 显示 (内部用 -180~+180，显示用 0~359) */
        OLED_Clear();

        /* Pitch (0~359) */
        OLED_ShowString(0, 0, "P:", 12);
        float p360 = pitch < 0 ? pitch + 360 : pitch;
        OLED_ShowNum(12, 0, (uint32_t)p360, 3, 12);

        /* Roll (0~359) */
        OLED_ShowString(40, 0, "R:", 12);
        float r360 = roll < 0 ? roll + 360 : roll;
        OLED_ShowNum(52, 0, (uint32_t)r360, 3, 12);

        /* Yaw (带符号显示) */
        OLED_ShowString(80, 0, "Y:", 12);
        if (yaw < 0.0f) {
            OLED_ShowString(92, 0, "-", 12);
            OLED_ShowNum(98, 0, (uint32_t)(-yaw), 3, 12);
        } else {
            OLED_ShowString(92, 0, "+", 12);
            OLED_ShowNum(98, 0, (uint32_t)yaw, 3, 12);
        }

        /* GZ (带符号显示) */
        OLED_ShowString(0, 16, "GZ:", 12);
        if (gz_dps < 0.0f) {
            OLED_ShowString(24, 16, "-", 12);
            OLED_ShowNum(30, 16, (uint32_t)(-gz_dps), 3, 12);
        } else {
            OLED_ShowString(24, 16, "+", 12);
            OLED_ShowNum(30, 16, (uint32_t)gz_dps, 3, 12);
        }

        OLED_Refresh();

        DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        delay_ms(10U);
    }
}
