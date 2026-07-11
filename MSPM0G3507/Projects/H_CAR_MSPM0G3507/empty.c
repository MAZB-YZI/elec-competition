#include "ti_msp_dl_config.h"
#include "../../../../Modules/Drivers/OLED/oled.h"
#include "../../../../Modules/Drivers/MPU6050/soft_i2c.h"
#include "delay.h"

/*
 * MPU6050 安全测试
 * 先验证 I2C 通信正常，再读取数据
 */

#define MPU6050_ADDR 0x68

int main(void)
{
    uint8_t who_am_i = 0;
    uint8_t ax_h = 0, ay_h = 0, az_h = 0;
    bool ok = false;

    SYSCFG_DL_init();

    /* 初始化 */
    SoftI2C_Init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 Safe", 16);
    OLED_Refresh();
    delay_ms(200U);

    /* 读取 WHO_AM_I */
    ok = SoftI2C_ReadReg(MPU6050_ADDR, 0x75, &who_am_i);
    OLED_Clear();
    OLED_ShowString(0, 0, "WHO:", 16);
    if (ok) {
        OLED_ShowNum(32, 0, who_am_i, 3, 16);
    } else {
        OLED_ShowString(32, 0, "FAIL", 16);
    }
    OLED_Refresh();
    delay_ms(500U);

    /* 唤醒 */
    SoftI2C_WriteReg(MPU6050_ADDR, 0x6B, 0x01);
    delay_ms(100U);

    /* 主循环：读取加速度高字节 */
    while (1) {
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3B, &ax_h);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3D, &ay_h);
        SoftI2C_ReadReg(MPU6050_ADDR, 0x3F, &az_h);

        OLED_Clear();
        OLED_ShowString(0, 0, "AX:", 16);
        OLED_ShowNum(24, 0, ax_h, 3, 16);
        OLED_ShowString(0, 16, "AY:", 16);
        OLED_ShowNum(24, 16, ay_h, 3, 16);
        OLED_ShowString(0, 32, "AZ:", 16);
        OLED_ShowNum(24, 32, az_h, 3, 16);
        OLED_Refresh();

        DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        delay_ms(200U);
    }
}
