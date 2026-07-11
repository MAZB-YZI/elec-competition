/*
 * H_CAR_MSPM0G3507 - 2024年全国大学生电子设计竞赛H题自动行驶小车
 *
 * 主控：MSPM0G3507
 * 电机驱动：TB6612
 * 传感器：MPU6050、灰度模块（队友负责）
 * 显示：OLED（I2C）
 * 通信：UART调试串口
 *
 * 引脚分配（按硬件手册 V3.1）：
 * - Motor1: PWMA=PA12, AIN1=PB17, AIN2=PB19
 * - Motor2: PWMB=PA13, BIN1=PA16, BIN2=PB24
 * - Encoder1: A=PA25, B=PA14
 * - Encoder2: A=PA26, B=PA27
 * - I2C0: SDA=PA28, SCL=PA31 (OLED + MPU6050)
 * - Buzzer: PA7
 * - LED: PB22 (板载LED)
 * - Debug UART: PB6(TX), PB7(RX)
 */

#include "ti_msp_dl_config.h"
#include "hcar_app.h"
#include "hcar_hal.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "buzzer.h"
#include "control.h"
#include "route_fsm.h"
#include "line_sensor.h"

/* 函数声明 */
void HCar_1msTickCallback(void);

/* SysConfig 生成的定时器中断处理函数 */
void TIMG0_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(SYS_TICK_INST)) {
        case DL_TIMER_IIDX_ZERO:
            HCar_1msTickCallback();
            break;
        default:
            break;
    }
}

/* HCarApp 1ms 回调函数 */
void HCar_1msTickCallback(void)
{
    HCarApp_Tick1msISR();
}

int main(void)
{
    /* SysConfig 初始化 */
    SYSCFG_DL_init();

    /* 初始化应用 */
    if (!HCarApp_Init() || !HCarHal_Start1msTick()) {
        Motor_Stop();
        for (;;) {
            __WFI();
        }
    }

    /* 主循环 */
    for (;;) {
        HCarApp_RunPending();
        __WFI();
    }
}
