#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "motor.h"
#include "buzzer.h"
#include "mpu6050.h"

#include <stdbool.h>
#include <stdint.h>

static volatile uint32_t g_ms_ticks;

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static void OLED_ShowSigned5(uint8_t x, uint8_t y, int32_t value)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        value = -value;
    } else {
        OLED_ShowString(x, y, "+", 12);
    }
    OLED_ShowNum((uint8_t)(x + 6U), y, (uint32_t)value, 5, 12);
}

static void show_boot(const char *line)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "BNO055 TEST", 16);
    OLED_ShowString(0, 20, line, 12);
    OLED_ShowString(0, 36, "PA0 PA1 I2C", 12);
    OLED_Refresh();
}

int main(void)
{
    uint32_t last_ms;
    uint32_t last_oled_ms;
    uint32_t sample_count = 0U;
    bool ok = true;

    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
    Motor_Init();
    Motor_Stop();
    Buzzer_Init();
    Buzzer_Stop();
    OLED_Init();

    show_boot("INIT...");

    if (!MPU6050_Init()) {
        OLED_Clear();
        OLED_ShowString(0, 0, "BNO INIT FAIL", 12);
        OLED_ShowString(0, 16, "A", 12);
        OLED_ShowNum(8, 16, MPU6050_GetI2CAddress(), 3, 12);
        OLED_ShowString(40, 16, "ID", 12);
        OLED_ShowNum(58, 16, MPU6050_GetWhoAmI(), 3, 12);
        OLED_ShowString(0, 32, "E", 12);
        OLED_ShowNum(8, 32, MPU6050_GetReadErrorCount(), 5, 12);
        OLED_ShowString(0, 48, "ADD GND=028", 12);
        OLED_Refresh();
        while (1) {
            Motor_Stop();
            Buzzer_Stop();
        }
    }

    show_boot("NDOF OK");
    delay_ms(300U);
    MPU6050_ResetYaw();
    last_ms = g_ms_ticks;
    last_oled_ms = g_ms_ticks;

    while (1) {
        uint32_t now = g_ms_ticks;
        uint32_t elapsed = now - last_ms;

        Motor_Stop();
        Buzzer_Stop();

        if (elapsed >= 10U) {
            last_ms = now;
            ok = MPU6050_UpdateYawOnly((float)elapsed / 1000.0f);
            if (ok) sample_count++;
        }

        if ((now - last_oled_ms) >= 200U) {
            int32_t h10 = (int32_t)(BNO055_GetHeading() * 10.0f);
            int32_t y10 = (int32_t)(MPU6050_GetYaw() * 10.0f);
            int32_t r10 = (int32_t)(MPU6050_GetRoll() * 10.0f);
            int32_t p10 = (int32_t)(MPU6050_GetPitch() * 10.0f);

            last_oled_ms = now;
            OLED_Clear();
            OLED_ShowString(0, 0, "A", 12);
            OLED_ShowNum(8, 0, MPU6050_GetI2CAddress(), 3, 12);
            OLED_ShowString(40, 0, "ID", 12);
            OLED_ShowNum(58, 0, MPU6050_GetWhoAmI(), 3, 12);
            OLED_ShowString(88, 0, ok ? "OK" : "ER", 12);

            OLED_ShowString(0, 13, "C", 12);
            OLED_ShowNum(8, 13, BNO055_GetCalibSys(), 1, 12);
            OLED_ShowNum(18, 13, BNO055_GetCalibGyro(), 1, 12);
            OLED_ShowNum(28, 13, BNO055_GetCalibAccel(), 1, 12);
            OLED_ShowNum(38, 13, BNO055_GetCalibMag(), 1, 12);
            OLED_ShowString(56, 13, "N", 12);
            OLED_ShowNum(64, 13, sample_count % 10000U, 4, 12);

            OLED_ShowString(0, 26, "H", 12);
            OLED_ShowSigned5(8, 26, h10);
            OLED_ShowString(70, 26, "Y", 12);
            OLED_ShowSigned5(78, 26, y10);

            OLED_ShowString(0, 39, "R", 12);
            OLED_ShowSigned5(8, 39, r10);
            OLED_ShowString(70, 39, "P", 12);
            OLED_ShowSigned5(78, 39, p10);

            OLED_ShowString(0, 52, "E", 12);
            OLED_ShowNum(8, 52, MPU6050_GetReadErrorCount(), 5, 12);
            OLED_ShowString(56, 52, "x0.1deg", 12);
            OLED_Refresh();
        }
    }
}
