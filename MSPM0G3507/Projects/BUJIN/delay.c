#include "delay.h"

#define SYS_FREQ_HZ 32000000U

void delay_ms(uint32_t ms)
{
    uint32_t tick = ms * (SYS_FREQ_HZ / 4000);
    for (volatile uint32_t i = 0; i < tick; ) i++;
}

void delay_us(uint32_t us)
{
    uint32_t tick = us * (SYS_FREQ_HZ / 4000000U);
    for (volatile uint32_t i = 0; i < tick; ) i++;
}
