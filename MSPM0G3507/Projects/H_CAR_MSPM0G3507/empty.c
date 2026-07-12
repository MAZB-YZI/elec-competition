#include "delay.h"
#include "mpu6050.h"
#include "ti_msp_dl_config.h"
#include "../../../Modules/Drivers/MPU6050/soft_i2c.h"
#include "../../../Modules/Drivers/OLED/oled.h"

static volatile uint32_t g_ms_ticks = 0U;

static void display_signed_int(uint8_t x, uint8_t y, int32_t value, uint8_t len)
{
    if (value < 0) {
        OLED_ShowString(x, y, "-", 12);
        OLED_ShowNum((uint8_t) (x + 8U), y, (uint32_t) (-value), len, 12);
    } else {
        OLED_ShowString(x, y, "+", 12);
        OLED_ShowNum((uint8_t) (x + 8U), y, (uint32_t) value, len, 12);
    }
}

static void display_signed_tenths(uint8_t x, uint8_t y, float value)
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

static bool read_who_retry(uint8_t *who)
{
    for (uint8_t attempt = 0U; attempt < 10U; ++attempt) {
        SoftI2C_Init();
        delay_ms(20U);
        if (SoftI2C_ProbeAddress(0x68U) &&
            SoftI2C_ReadReg(0x68U, 0x75U, who)) {
            return true;
        }
        delay_ms(30U);
    }

    return false;
}

void SYS_TICK_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(SYS_TICK_INST)) {
        case DL_TIMER_IIDX_ZERO:
            ++g_ms_ticks;
            break;
        default:
            break;
    }
}

int main(void)
{
    float yaw = 0.0f;
    float gz = 0.0f;
    float bias = 0.0f;
    uint8_t who = 0U;
    uint32_t err_count = 0U;
    bool who_ok = false;
    uint32_t last_update_ms = 0U;
    uint32_t last_display_ms = 0U;
    uint32_t now_ms = 0U;
    uint32_t dt_ms = 0U;

    SYSCFG_DL_init();
    SoftI2C_Init();
    OLED_Init();
    NVIC_ClearPendingIRQ(SYS_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(SYS_TICK_INST_INT_IRQN);

    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 TEST", 12);
    OLED_ShowString(0, 16, "READ WHO...", 12);
    OLED_Refresh();
    delay_ms(300U);

    who_ok = read_who_retry(&who);
    if (!who_ok) {
        OLED_Clear();
        OLED_ShowString(0, 0, "WHO FAIL", 12);
        OLED_ShowString(0, 16, "ADDR 0x68", 12);
        OLED_ShowString(0, 32, "RETRY=10", 12);
        OLED_Refresh();
        while (1) {
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
            delay_ms(200U);
        }
    }

    if (!MPU6050_Init()) {
        OLED_Clear();
        OLED_ShowString(0, 0, "INIT FAIL", 12);
        OLED_ShowString(0, 16, "WHO:", 12);
        OLED_ShowNum(36, 16, who, 3, 12);
        OLED_Refresh();
        while (1) {
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
            delay_ms(200U);
        }
    }

    OLED_Clear();
    OLED_ShowString(0, 0, "CALIBRATING", 12);
    OLED_ShowString(0, 16, "KEEP STILL 2S", 12);
    OLED_ShowString(0, 32, "WHO:", 12);
    OLED_ShowNum(36, 32, who, 3, 12);
    OLED_Refresh();

    delay_ms(500U);

    if (!MPU6050_CalibrateGyro(1000U)) {
        OLED_Clear();
        OLED_ShowString(0, 0, "CAL FAIL", 12);
        OLED_ShowString(0, 16, "ERR:", 12);
        OLED_ShowNum(36, 16, MPU6050_GetReadErrorCount(), 5, 12);
        OLED_Refresh();
        while (1) {
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
            delay_ms(200U);
        }
    }

    last_update_ms = g_ms_ticks;
    last_display_ms = g_ms_ticks;

    while (1) {
        now_ms = g_ms_ticks;
        dt_ms = now_ms - last_update_ms;

        if (dt_ms > 0U) {
            last_update_ms = now_ms;
        }

        if ((dt_ms > 0U) && !MPU6050_Update((float) dt_ms / 1000.0f)) {
            OLED_Clear();
            OLED_ShowString(0, 0, "READ FAIL", 12);
            OLED_ShowString(0, 16, "ERR:", 12);
            OLED_ShowNum(36, 16, MPU6050_GetReadErrorCount(), 5, 12);
            OLED_Refresh();
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
            continue;
        }

        yaw = MPU6050_GetYaw();
        gz = MPU6050_GetGyroZ();
        bias = MPU6050_GetGyroZBias();
        err_count = MPU6050_GetReadErrorCount();

        if ((now_ms - last_display_ms) >= 100U) {
            last_display_ms = now_ms;
            OLED_Clear();
            OLED_ShowString(0, 0, "WHO:", 12);
            OLED_ShowNum(36, 0, who, 3, 12);
            OLED_ShowString(64, 0, "ER:", 12);
            OLED_ShowNum(88, 0, err_count, 4, 12);

            OLED_ShowString(0, 16, "GZ:", 12);
            display_signed_tenths(24, 16, gz);
            OLED_ShowString(64, 16, "BZ:", 12);
            display_signed_int(88, 16, (int32_t) bias, 3);

            OLED_ShowString(0, 32, "YAW:", 12);
            display_signed_tenths(32, 32, yaw);

            OLED_ShowString(0, 48, "TURN BOARD", 12);
            OLED_ShowString(0, 60, "TICK=1ms", 12);

            OLED_Refresh();
            DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        }
    }
}
