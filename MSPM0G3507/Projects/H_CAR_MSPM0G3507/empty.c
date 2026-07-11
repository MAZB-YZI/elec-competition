#include "ti_msp_dl_config.h"
#include "../../../../Modules/Drivers/OLED/oled.h"
#include "../../../../Modules/Drivers/MPU6050/soft_i2c.h"
#include "delay.h"

/*
 * MPU6050 完整测试
 */

#define MPU6050_ADDR 0x68

int main(void)
{
    uint8_t ax_h, ax_l, ay_h, ay_l, az_h, az_l;
    bool ok = false;

    SYSCFG_DL_init();

    /* 初始化软件 I2C */
    SoftI2C_Init();

    /* 初始化 OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 Test", 16);
    OLED_Refresh();

    /* 唤醒 MPU6050 */
    SoftI2C_WriteReg(MPU6050_ADDR, 0x6B, 0x00);
    delay_ms(100U);

    /* 主循环 */
    while (1) {
        /* 读取加速度计数据 */
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3B, &ax_h);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3C, &ax_l);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3D, &ay_h);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3E, &ay_l);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3F, &az_h);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x40, &az_l);

        /* 合成 16 位值 */
        int16_t ax = (int16_t)((ax_h << 8) | ax_l);
        int16_t ay = (int16_t)((ay_h << 8) | ay_l);
        int16_t az = (int16_t)((az_h << 8) | az_l);

        /* 显示 */
        OLED_Clear();
        OLED_ShowString(0, 0, "Accel:", 16);
        OLED_ShowString(0, 16, "X:", 16);
        OLED_ShowNum(16, 16, (uint32_t)(ax + 32768), 5, 16);
        OLED_ShowString(0, 32, "Y:", 16);
        OLED_ShowNum(16, 32, (uint32_t)(ay + 32768), 5, 16);
        OLED_ShowString(0, 48, "Z:", 16);
        OLED_ShowNum(16, 48, (uint32_t)(az + 32768), 5, 16);
        OLED_Refresh();

        DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        delay_ms(200U);
    }
}
