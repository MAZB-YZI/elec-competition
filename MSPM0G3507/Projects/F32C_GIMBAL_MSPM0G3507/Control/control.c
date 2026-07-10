#include "control.h"

extern int Motor1_T_Position, Motor2_T_Position;

/* TIMER0 周期中断：参考位置闭环例程，在定时中断内检测按键并修改目标位置。 */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        LED_Flash(100);

        /* 按键每按下一次，两个电机目标位置同时增加 900，即 90.0 度。 */
        if (click() == 1)
        {
            Motor1_T_Position += 900;
            Motor2_T_Position += 900;
        }
    }
}