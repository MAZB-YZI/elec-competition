#include "ti_msp_dl_config.h"
#include "../../../../Modules/Drivers/OLED/oled.h"
#include "delay.h"

/*
 * 电机测试程序
 * - OLED 显示测试状态
 * - Motor1 和 Motor2 同时正转
 * - LED 闪烁
 */

int main(void)
{
    SYSCFG_DL_init();

    /* 初始化 OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "Motor Test", 16);
    OLED_ShowString(0, 16, "Motor1: ON", 16);
    OLED_ShowString(0, 32, "Motor2: ON", 16);
    OLED_Refresh();

    /* 电机方向：AIN1=0, AIN2=1 = 正转 */
    DL_GPIO_clearPins(MOTOR1_AIN1_PORT, MOTOR1_AIN1_AIN1_PIN);
    DL_GPIO_setPins(MOTOR1_AIN2_PORT, MOTOR1_AIN2_AIN2_PIN);

    /* 电机方向：BIN1=0, BIN2=1 = 正转 */
    DL_GPIO_clearPins(MOTOR2_BIN1_PORT, MOTOR2_BIN1_BIN1_PIN);
    DL_GPIO_setPins(MOTOR2_BIN2_PORT, MOTOR2_BIN2_BIN2_PIN);

    while (1) {
        /* LED 闪烁 */
        DL_GPIO_togglePins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
        delay_ms(500U);
    }
}
