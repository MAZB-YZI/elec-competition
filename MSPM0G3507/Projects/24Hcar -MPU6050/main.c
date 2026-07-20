#include "ti_msp_dl_config.h"

#include "bluetooth.h"
#include "delay.h"
#include "mpu6050.h"
#include "oled.h"
#include "soft_i2c.h"

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_UPDATE_PERIOD_MS 10U
#define OLED_REFRESH_PERIOD_MS   200U

static volatile uint32_t g_ms_ticks;

void SysTick_Handler(void)
{
    ++g_ms_ticks;
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void Buzzer_Off(void)
{
    /*
     * The buzzer on this car is active-low.  SysConfig may leave the GPIO low
     * after reset, so force it high before the MPU6050 stability test starts.
     */
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
}

static void OLED_ShowSigned(uint8_t x, uint8_t y, int32_t value, uint8_t len)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        value = -value;
    } else {
        OLED_ShowString(x, y, "+", 12);
    }
    OLED_ShowNum((uint8_t) (x + 6U), y, (uint32_t) value, len, 12);
}

static void ShowMpuStatus(bool init_ok, bool cal_ok)
{
    int32_t yaw_deg = (int32_t) MPU6050_GetYaw();
    int32_t gz_dps = (int32_t) MPU6050_GetGyroZ();
    int32_t bias = (int32_t) MPU6050_GetGyroZBias();

    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 TEST", 12);

    OLED_ShowString(0, 12, "WHO:", 12);
    OLED_ShowNum(30, 12, MPU6050_GetWhoAmI(), 3, 12);
    OLED_ShowString(66, 12, init_ok ? "INIT OK" : "INIT NG", 12);

    OLED_ShowString(0, 24, "CAL:", 12);
    OLED_ShowString(30, 24, cal_ok ? "OK" : "NG", 12);
    OLED_ShowString(54, 24, "ERR:", 12);
    OLED_ShowNum(84, 24, MPU6050_GetReadErrorCount(), 4, 12);

    OLED_ShowString(0, 36, "GZ", 12);
    OLED_ShowSigned(18, 36, gz_dps, 4);
    OLED_ShowString(72, 36, "B", 12);
    OLED_ShowSigned(84, 36, bias, 4);

    OLED_ShowString(0, 52, "YAW", 12);
    OLED_ShowSigned(24, 52, yaw_deg, 4);
    OLED_ShowString(60, 52, "deg", 12);
    OLED_Refresh();
}

int main(void)
{
    bool init_ok;
    bool cal_ok = false;
    uint32_t last_mpu_ms;
    uint32_t last_oled_ms;

    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
    Buzzer_Off();
    delay_cycles(3200000U);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 TEST", 16);
    OLED_ShowString(0, 20, "Keep car still", 12);
    OLED_Refresh();

    SoftI2C_Init();
    init_ok = MPU6050_Init();
    if (init_ok) {
        OLED_ShowString(0, 36, "Calibrating...", 12);
        OLED_Refresh();
        delay_ms(500U);
        cal_ok = MPU6050_CalibrateGyro(500U);
        MPU6050_ResetYaw();
    }

    BT_Send(init_ok ? "MPU6050 init ok\r\n" : "MPU6050 init failed\r\n");
    last_mpu_ms = millis();
    last_oled_ms = millis();

    while (1) {
        uint32_t now = millis();

        if (init_ok && cal_ok &&
            ((uint32_t) (now - last_mpu_ms) >= MPU6050_UPDATE_PERIOD_MS)) {
            uint32_t elapsed_ms = (uint32_t) (now - last_mpu_ms);
            last_mpu_ms = now;
            (void) MPU6050_Update((float) elapsed_ms / 1000.0f);
        }

        if ((uint32_t) (now - last_oled_ms) >= OLED_REFRESH_PERIOD_MS) {
            last_oled_ms = now;
            ShowMpuStatus(init_ok, cal_ok);
        }
    }
}
