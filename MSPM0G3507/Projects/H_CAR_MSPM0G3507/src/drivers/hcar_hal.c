/*
 * hcar_hal.c - H_CAR 硬件抽象层实现
 *
 * 使用 SysConfig 生成的宏名称
 */

#include "hcar_hal.h"
#include "ti_msp_dl_config.h"

/* 初始化硬件 */
bool HCarHal_Init(void)
{
    /* 配置编码器 A 相为输入 */
    DL_GPIO_initDigitalInput(ENCODER1_A_ENC1_A_IOMUX);
    DL_GPIO_initDigitalInput(ENCODER2_A_ENC2_A_IOMUX);

    /* 使能编码器 A 相中断 */
    DL_GPIO_enableInterrupt(GPIOA, ENCODER1_A_ENC1_A_PIN);
    DL_GPIO_enableInterrupt(GPIOB, ENCODER2_A_ENC2_A_PIN);

    /* 使能 GPIO 中断 */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    return true;
}

/* 启动 1ms 时基 */
bool HCarHal_Start1msTick(void)
{
    /* SysConfig 已经配置好定时器，只需要使能中断 */
    NVIC_EnableIRQ(SYS_TICK_INST_INT_IRQN);
    return true;
}

/* 空闲处理 */
void HCarHal_Idle(void)
{
    __WFI();
}

/* 设置电机方向 */
void HCarHal_SetMotorDirection(bool left, bool in1, bool in2)
{
    if (left) {
        if (in1) DL_GPIO_setPins(MOTOR1_AIN1_PORT, MOTOR1_AIN1_AIN1_PIN);
        else DL_GPIO_clearPins(MOTOR1_AIN1_PORT, MOTOR1_AIN1_AIN1_PIN);
        if (in2) DL_GPIO_setPins(MOTOR1_AIN2_PORT, MOTOR1_AIN2_AIN2_PIN);
        else DL_GPIO_clearPins(MOTOR1_AIN2_PORT, MOTOR1_AIN2_AIN2_PIN);
    } else {
        if (in1) DL_GPIO_setPins(MOTOR2_BIN1_PORT, MOTOR2_BIN1_BIN1_PIN);
        else DL_GPIO_clearPins(MOTOR2_BIN1_PORT, MOTOR2_BIN1_BIN1_PIN);
        if (in2) DL_GPIO_setPins(MOTOR2_BIN2_PORT, MOTOR2_BIN2_BIN2_PIN);
        else DL_GPIO_clearPins(MOTOR2_BIN2_PORT, MOTOR2_BIN2_BIN2_PIN);
    }
}

/* 设置电机占空比 - TODO: 需要配置 PWM 定时器 */
void HCarHal_SetMotorDuty(bool left, uint16_t duty)
{
    /* TODO: 实现 PWM 控制 */
    (void)left;
    (void)duty;
}

/* 获取电机 PWM 周期 */
uint16_t HCarHal_GetMotorPeriod(void)
{
    return 4000;  /* 与 SysConfig 配置的周期一致 */
}

/* 读取编码器 B 相 */
bool HCarHal_ReadEncoderB(bool left)
{
    if (left) {
        return DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_14);  /* Motor1 Encoder B = PA14 */
    } else {
        return DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_27);  /* Motor2 Encoder B = PA27 */
    }
}

/* 设置蜂鸣器 */
void HCarHal_SetBuzzer(bool on)
{
    if (on) DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_CTRL_PIN);
    else DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_CTRL_PIN);
}

/* 设置状态 LED */
void HCarHal_SetStatusLed(bool on)
{
    if (on) DL_GPIO_setPins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
    else DL_GPIO_clearPins(LED_STATUS_PORT, LED_STATUS_LED_PIN);
}

/* I2C 写入 */
bool HCarHal_I2CWrite(uint8_t address, const uint8_t *data, size_t length,
    uint32_t timeout_ticks)
{
    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_setTargetAddress(I2C_OLED_INST, address);

    for (size_t i = 0; i < length; i++) {
        DL_I2C_transmitControllerData(I2C_OLED_INST, data[i]);
    }

    /* 等待传输完成 */
    uint32_t timeout = timeout_ticks;
    while (DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout-- == 0) return false;
    }

    return true;
}

/* I2C 写入并读取 */
bool HCarHal_I2CWriteRead(uint8_t address, const uint8_t *tx, size_t tx_length,
    uint8_t *rx, size_t rx_length, uint32_t timeout_ticks)
{
    /* 写入寄存器地址 */
    if (!HCarHal_I2CWrite(address, tx, tx_length, timeout_ticks)) {
        return false;
    }

    /* 读取数据 */
    DL_I2C_flushControllerRXFIFO(I2C_OLED_INST);
    DL_I2C_setTargetAddress(I2C_OLED_INST, address);

    for (size_t i = 0; i < rx_length; i++) {
        rx[i] = DL_I2C_receiveControllerData(I2C_OLED_INST);
    }

    /* 等待传输完成 */
    uint32_t timeout = timeout_ticks;
    while (DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout-- == 0) return false;
    }

    return true;
}
