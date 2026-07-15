#include "delay.h"

#define SYS_FREQ_HZ 32000000U

void delay_ms(uint32_t ms)
{
    uint32_t tick = ms * (SYS_FREQ_HZ / 4000);
    for (volatile uint32_t i = 0; i < tick; ) i++;
}
