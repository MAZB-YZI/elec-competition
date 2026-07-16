/*
 * hcar_hal.c - H_CAR 板级适配文件
 * JY61P 通过 UART0 (PA0/PA1) 通信，不需要 I2C
 */

#include "hcar_hal.h"
#include "encoder.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

bool HCarHal_Init(void)
{
    /* 初始化编码器中断 */
    DL_GPIO_clearInterruptStatus(GPIOA,
        ENCODER1_A_ENC1_A_PIN | ENCODER2_A_ENC2_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER1_A_ENC1_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER2_A_ENC2_A_PIN);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    return true;
}

bool HCarHal_Start1msTick(void)
{
    NVIC_ClearPendingIRQ(SYS_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(SYS_TICK_INST_INT_IRQN);
    return true;
}

void HCarHal_Idle(void)
{
    __WFI();
}

void HCarHal_SetMotorDirection(bool left, bool in1, bool in2)
{
    if (left) {
        if (in1) DL_GPIO_setPins(MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_L_PORT, MOTOR_DIR_L_L_DIR1_PIN);

        if (in2) DL_GPIO_setPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_L_DIR2_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_L_DIR2_PIN);
    } else {
        if (in1) DL_GPIO_setPins(MOTOR_DIR_R_PORT, MOTOR_DIR_R_R_DIR1_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_R_PORT, MOTOR_DIR_R_R_DIR1_PIN);

        if (in2) DL_GPIO_setPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_R_DIR2_PIN);
        else DL_GPIO_clearPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_R_DIR2_PIN);
    }
}

void HCarHal_SetMotorDuty(bool left, uint16_t duty)
{
    (void) left;
    (void) duty;
}

uint16_t HCarHal_GetMotorPeriod(void)
{
    return 4000U;
}

bool HCarHal_ReadEncoderB(bool left)
{
    if (left) {
        return (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_27) != 0U);
    }
    return (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_14) != 0U);
}

void HCarHal_SetBuzzer(bool on)
{
    /* 蜂鸣器低电平有效：on=true → 拉低响，on=false → 拉高停 */
    if (on) DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_CTRL_PIN);
    else DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_CTRL_PIN);
}

bool EncoderHal_ReadPhaseB(bool left)
{
    return HCarHal_ReadEncoderB(left);
}

void BuzzerHal_SetOutput(bool on)
{
    HCarHal_SetBuzzer(on);
}

void HCarHal_SetStatusLed(bool on)
{
    if (on) DL_GPIO_setPins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
    else DL_GPIO_clearPins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
}

/* ========== 编码器中断 ========== */

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        ENCODER1_A_ENC1_A_PIN | ENCODER2_A_ENC2_A_PIN);

    if ((pending & ENCODER1_A_ENC1_A_PIN) != 0U) {
        Encoder_OnLeftAEdge();
    }
    if ((pending & ENCODER2_A_ENC2_A_PIN) != 0U) {
        Encoder_OnRightAEdge();
    }

    DL_GPIO_clearInterruptStatus(GPIOA, pending);
}
