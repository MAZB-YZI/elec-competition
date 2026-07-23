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

static void OLED_ShowSigned4(uint8_t x, uint8_t y, int32_t value)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        value = -value;
    } else {
        OLED_ShowString(x, y, "+", 12);
    }
    OLED_ShowNum((uint8_t)(x + 6U), y, (uint32_t)value, 4, 12);
}

static void show_boot(const char *line2)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "ICM42688P", 16);
    OLED_ShowString(0, 20, line2, 12);
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
        OLED_ShowString(0, 0, "ICM INIT FAIL", 12);
        OLED_ShowString(0, 16, "A:", 12);
        OLED_ShowNum(18, 16, MPU6050_GetI2CAddress(), 3, 12);
        OLED_ShowString(0, 32, "W:", 12);
        OLED_ShowNum(18, 32, MPU6050_GetWhoAmI(), 3, 12);
        OLED_ShowString(0, 48, "E:", 12);
        OLED_ShowNum(18, 48, MPU6050_GetReadErrorCount(), 5, 12);
        OLED_Refresh();
        while (1) {
            Motor_Stop();
            Buzzer_Stop();
        }
    }

    show_boot("KEEP STILL");
    delay_ms(500U);

    if (!MPU6050_CalibrateGyro(1200U)) {
        OLED_Clear();
        OLED_ShowString(0, 0, "CAL FAIL", 12);
        OLED_ShowString(0, 16, "W:", 12);
        OLED_ShowNum(18, 16, MPU6050_GetWhoAmI(), 3, 12);
        OLED_ShowString(0, 32, "E:", 12);
        OLED_ShowNum(18, 32, MPU6050_GetReadErrorCount(), 5, 12);
        OLED_Refresh();
        while (1) {
            Motor_Stop();
            Buzzer_Stop();
        }
    }

    MPU6050_ResetYaw();
    last_ms = g_ms_ticks;
    last_oled_ms = g_ms_ticks;

    OLED_Clear();
    OLED_ShowString(0, 0, "ICM TEST OK", 12);
    OLED_Refresh();

    while (1) {
        uint32_t now = g_ms_ticks;
        uint32_t elapsed = now - last_ms;

        Motor_Stop();
        Buzzer_Stop();

        if (elapsed > 0U) {
            last_ms = now;
            ok = MPU6050_UpdateYawOnly((float)elapsed / 1000.0f);
            if (ok) {
                sample_count++;
            }
        }

        if ((now - last_oled_ms) >= 200U) {
            int32_t yaw10 = (int32_t)(MPU6050_GetYaw() * 10.0f);
            int32_t gz10 = (int32_t)(MPU6050_GetGyroZ() * 10.0f);
            int32_t bias = (int32_t)MPU6050_GetGyroZBias();

            last_oled_ms = now;
            OLED_Clear();
            OLED_ShowString(0, 0, "A", 12);
            OLED_ShowNum(8, 0, MPU6050_GetI2CAddress(), 3, 12);
            OLED_ShowString(40, 0, "W", 12);
            OLED_ShowNum(48, 0, MPU6050_GetWhoAmI(), 3, 12);
            OLED_ShowString(84, 0, ok ? "OK" : "ER", 12);

            OLED_ShowString(0, 16, "N", 12);
            OLED_ShowNum(8, 16, sample_count % 10000U, 4, 12);
            OLED_ShowString(56, 16, "E", 12);
            OLED_ShowNum(64, 16, MPU6050_GetReadErrorCount(), 5, 12);

            OLED_ShowString(0, 32, "Z", 12);
            OLED_ShowSigned4(8, 32, gz10);
            OLED_ShowString(64, 32, "B", 12);
            OLED_ShowSigned4(72, 32, bias);

            OLED_ShowString(0, 48, "Y", 12);
            OLED_ShowSigned4(8, 48, yaw10);
            OLED_ShowString(70, 48, "x0.1", 12);
            OLED_Refresh();
        }
    }
}

