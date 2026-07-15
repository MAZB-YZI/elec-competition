#include "buzzer.h"

static volatile uint32_t remaining;

void Buzzer_Init(void)
{
    Buzzer_Stop();
}

void Buzzer_Beep(uint32_t duration_ms)
{
    remaining = duration_ms;
    BuzzerHal_SetOutput(duration_ms != 0U);
}

void Buzzer_Update1ms(void)
{
    if ((remaining != 0U) && (--remaining == 0U)) {
        BuzzerHal_SetOutput(false);
    }
}

void Buzzer_Stop(void)
{
    remaining = 0U;
    BuzzerHal_SetOutput(false);
}
