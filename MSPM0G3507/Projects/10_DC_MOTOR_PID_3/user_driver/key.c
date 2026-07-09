#include "key.h"
extern int status;

uint8_t get_key_state(uint32_t key) {
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key); //0x00000040 0b01000000 PB6 0~31
    if((high_bits & key) != 0) return 1;
    else return 0;
}

uint32_t counter_1_A = 0;

void GROUP1_IRQHandler()
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_KEY9_IIDX:
        /* code */
        status = (status + 1) % 3;
        break;
    case KEY_KEY10_IIDX:
        status = (status + 3 -1) % 3;
        /* code */
        break;
    
    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_AA_IIDX:
        /* code */
        counter_1_A ++;
        break;
    
    default:
        break;
    }

}



