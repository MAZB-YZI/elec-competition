#include "delay.h"
#include <ti/devices/msp/msp.h>

void delay_us(uint32_t us)
{
    SysTick->LOAD = (CPUCLK_FREQ / 1000000) * us - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    SysTick->CTRL = 0;
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

