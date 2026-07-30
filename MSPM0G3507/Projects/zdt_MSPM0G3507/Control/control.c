#include "control.h"

extern int Motor1_T_Position, Motor2_T_Position;

/* TIMER0 定时中断（暂时禁用按键功能，避免误触发）
 * DISABLED — new TIMER handler in empty.c */
#if 0
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        LED_Flash(100);

        /* 按键功能暂时禁用，避免误触发修改目标位置
        if (click() == 1)
        {
            Motor1_T_Position += 900;
            Motor2_T_Position += 900;
        }
        */
    }
}
#endif /* TIMER_0_INST_IRQHandler disabled — new handler in empty.c */
