#include "key.h"

volatile int32_t counter_1_A = 0;
volatile int32_t counter_2_A = 0;

uint8_t keyValue(void)
{
    return (DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) & KEY_key_PIN) > 0 ? 0 : 1;
}

u8 click(void)
{
    static u8 flag_key = 1;
    if (flag_key && keyValue() == 1) {
        flag_key = 0;
        return 1;
    } else if (keyValue() == 0) {
        flag_key = 1;
    }
    return 0;
}

/* GPIO interrupt handler */
void GROUP1_IRQHandler(void)
{
    DL_GPIO_getPendingInterrupt(GPIOB);

    DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(GPIOA);
    if (iidx == DC_MOTOR_AA_IIDX) {
        if (DL_GPIO_readPins(DC_MOTOR_AB_PORT, DC_MOTOR_AB_PIN) != 0U) {
            counter_1_A--;
        } else {
            counter_1_A++;
        }
    } else if (iidx == DC_MOTOR_BA_IIDX) {
        if (DL_GPIO_readPins(DC_MOTOR_BB_PORT, DC_MOTOR_BB_PIN) != 0U) {
            counter_2_A--;
        } else {
            counter_2_A++;
        }
    }
}
