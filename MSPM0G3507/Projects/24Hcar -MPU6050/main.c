#include "ti_msp_dl_config.h"

#include "bluetooth.h"
#include "delay.h"
#include "mpu6050.h"
#include "oled.h"
#include "soft_i2c.h"

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR             0x68U
#define MPU6050_WHO_AM_I_REG     0x75U
#define OLED_REFRESH_PERIOD_MS   100U

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

static void OLED_ShowSignedTenths(uint8_t x, uint8_t y, float value)
{
    int32_t scaled = (int32_t) (value * 10.0f);
    uint32_t magnitude;

    if (scaled < 0) {
        OLED_ShowString(x, y, "-", 12);
        magnitude = (uint32_t) (-scaled);
    } else {
        OLED_ShowString(x, y, "+", 12);
        magnitude = (uint32_t) scaled;
    }

    OLED_ShowNum((uint8_t) (x + 8U), y, magnitude / 10U, 3, 12);
    OLED_ShowString((uint8_t) (x + 32U), y, ".", 12);
    OLED_ShowNum((uint8_t) (x + 40U), y, magnitude % 10U, 1, 12);
}

static bool ReadWhoRetry(uint8_t *who)
{
    if (who == 0) {
        return false;
    }

    for (uint8_t attempt = 0U; attempt < 10U; ++attempt) {
        SoftI2C_Init();
        delay_ms(20U);
        if (SoftI2C_ProbeAddress(MPU6050_ADDR) &&
            SoftI2C_ReadReg(MPU6050_ADDR, MPU6050_WHO_AM_I_REG, who)) {
            return true;
        }
        delay_ms(30U);
    }

    return false;
}

static void ShowMpuStatus(uint8_t who, bool init_ok, bool cal_ok)
{
    int32_t bias = (int32_t) MPU6050_GetGyroZBias();

    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 TEST", 12);

    OLED_ShowString(0, 12, "WHO:", 12);
    OLED_ShowNum(30, 12, who, 3, 12);
    OLED_ShowString(66, 12, init_ok ? "INIT OK" : "INIT NG", 12);

    OLED_ShowString(0, 24, "CAL:", 12);
    OLED_ShowString(30, 24, cal_ok ? "OK" : "NG", 12);
    OLED_ShowString(54, 24, "ERR:", 12);
    OLED_ShowNum(84, 24, MPU6050_GetReadErrorCount(), 4, 12);

    OLED_ShowString(0, 36, "GZ", 12);
    OLED_ShowSignedTenths(18, 36, MPU6050_GetGyroZ());
    OLED_ShowString(72, 36, "B", 12);
    OLED_ShowSigned(84, 36, bias, 4);

    OLED_ShowString(0, 52, "YAW", 12);
    OLED_ShowSignedTenths(32, 52, MPU6050_GetYaw());
    OLED_Refresh();
}

int main(void)
{
    uint8_t who = 0U;
    bool who_ok;
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
    OLED_ShowString(0, 20, "READ WHO...", 12);
    OLED_Refresh();

    who_ok = ReadWhoRetry(&who);
    if (!who_ok) {
        OLED_Clear();
        OLED_ShowString(0, 0, "WHO FAIL", 12);
        OLED_ShowString(0, 16, "ADDR 0x68", 12);
        OLED_ShowString(0, 32, "CHECK PA0 PA1", 12);
        OLED_Refresh();
        while (1) {
        }
    }

    init_ok = MPU6050_Init();
    if (init_ok) {
        who = MPU6050_GetWhoAmI();
        OLED_Clear();
        OLED_ShowString(0, 0, "CALIBRATING", 12);
        OLED_ShowString(0, 16, "KEEP STILL", 12);
        OLED_ShowString(0, 32, "WHO:", 12);
        OLED_ShowNum(36, 32, who, 3, 12);
        OLED_Refresh();
        OLED_ShowString(0, 36, "Calibrating...", 12);
        OLED_Refresh();
        delay_ms(500U);
        cal_ok = MPU6050_CalibrateGyro(1000U);
        MPU6050_ResetYaw();
    }

    BT_Send(init_ok ? "MPU6050 init ok\r\n" : "MPU6050 init failed\r\n");
    last_mpu_ms = millis();
    last_oled_ms = millis();

    while (1) {
        uint32_t now = millis();

        if (init_ok && cal_ok && ((uint32_t) (now - last_mpu_ms) > 0U)) {
            uint32_t elapsed_ms = (uint32_t) (now - last_mpu_ms);
            last_mpu_ms = now;
            (void) MPU6050_Update((float) elapsed_ms / 1000.0f);
        }

        if ((uint32_t) (now - last_oled_ms) >= OLED_REFRESH_PERIOD_MS) {
            last_oled_ms = now;
            ShowMpuStatus(who, init_ok, cal_ok);
        }
    }
}
