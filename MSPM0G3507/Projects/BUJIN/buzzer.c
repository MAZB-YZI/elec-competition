#include "buzzer.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_buzzer_ticks;

static void Buzzer_Set(bool on)
{
    if (on) {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
    } else {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
    }
}

void Buzzer_Init(void)
{
    Buzzer_Stop();
}

void Buzzer_Beep(uint32_t duration_ms)
{
    g_buzzer_ticks = (duration_ms + 4U) / 5U;
    Buzzer_Set(g_buzzer_ticks != 0U);
}

void Buzzer_Update5ms(void)
{
    if (g_buzzer_ticks == 0U) {
        return;
    }

    g_buzzer_ticks--;
    if (g_buzzer_ticks == 0U) {
        Buzzer_Set(false);
    }
}

void Buzzer_Stop(void)
{
    g_buzzer_ticks = 0U;
    Buzzer_Set(false);
}
