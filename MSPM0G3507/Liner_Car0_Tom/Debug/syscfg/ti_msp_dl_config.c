/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gCTRL_TIMERBackup;
DL_UART_Main_backupConfig gUART_PBBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_MOTOR_init();
    SYSCFG_DL_CTRL_TIMER_init();
    SYSCFG_DL_I2C_BUS_init();
    SYSCFG_DL_UART_CONSOLE_init();
    SYSCFG_DL_UART_EXT_init();
    SYSCFG_DL_UART_PB_init();
    /* Ensure backup structures have no valid state */

	gCTRL_TIMERBackup.backupRdy 	= false;
	gUART_PBBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(CTRL_TIMER_INST, &gCTRL_TIMERBackup);
	retStatus &= DL_UART_Main_saveConfiguration(UART_PB_INST, &gUART_PBBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(CTRL_TIMER_INST, &gCTRL_TIMERBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(UART_PB_INST, &gUART_PBBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(PWM_MOTOR_INST);
    DL_TimerG_reset(CTRL_TIMER_INST);
    DL_I2C_reset(I2C_BUS_INST);
    DL_UART_Main_reset(UART_CONSOLE_INST);
    DL_UART_Main_reset(UART_EXT_INST);
    DL_UART_Main_reset(UART_PB_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(PWM_MOTOR_INST);
    DL_TimerG_enablePower(CTRL_TIMER_INST);
    DL_I2C_enablePower(I2C_BUS_INST);
    DL_UART_Main_enablePower(UART_CONSOLE_INST);
    DL_UART_Main_enablePower(UART_EXT_INST);
    DL_UART_Main_enablePower(UART_PB_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C0_IOMUX,GPIO_PWM_MOTOR_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C1_IOMUX,GPIO_PWM_MOTOR_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);

    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_I2C_BUS_IOMUX_SDA, GPIO_I2C_BUS_IOMUX_SDA_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_I2C_BUS_IOMUX_SCL, GPIO_I2C_BUS_IOMUX_SCL_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_BUS_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_BUS_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_CONSOLE_IOMUX_TX, GPIO_UART_CONSOLE_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_CONSOLE_IOMUX_RX, GPIO_UART_CONSOLE_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_EXT_IOMUX_TX, GPIO_UART_EXT_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_EXT_IOMUX_RX, GPIO_UART_EXT_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_PB_IOMUX_TX, GPIO_UART_PB_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_PB_IOMUX_RX, GPIO_UART_PB_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(BUZZER_BUZZER_PIN_IOMUX);

    DL_GPIO_initDigitalOutput(GRAY_SENSOR_GRAY_CLK_IOMUX);

    DL_GPIO_initDigitalInput(GRAY_SENSOR_GRAY_DAT_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_L_DIR1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_L_DIR2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_R_DIR1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_R_DIR2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PA17_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PA18_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PA21_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PA22_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PB1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PB10_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PB11_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_POOL_GPIO_PB14_IOMUX);

    DL_GPIO_initDigitalInputFeatures(ENCODER_ENC_A1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_ENC_A2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_ENC_B1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_ENC_B2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, BUZZER_BUZZER_PIN_PIN |
		GRAY_SENSOR_GRAY_CLK_PIN |
		MOTOR_DIR_R_DIR1_PIN |
		GPIO_POOL_GPIO_PA17_PIN |
		GPIO_POOL_GPIO_PA18_PIN |
		GPIO_POOL_GPIO_PA21_PIN |
		GPIO_POOL_GPIO_PA22_PIN);
    DL_GPIO_enableOutput(GPIOA, BUZZER_BUZZER_PIN_PIN |
		GRAY_SENSOR_GRAY_CLK_PIN |
		MOTOR_DIR_R_DIR1_PIN |
		GPIO_POOL_GPIO_PA17_PIN |
		GPIO_POOL_GPIO_PA18_PIN |
		GPIO_POOL_GPIO_PA21_PIN |
		GPIO_POOL_GPIO_PA22_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_14_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_27_EDGE_RISE |
		DL_GPIO_PIN_26_EDGE_RISE |
		DL_GPIO_PIN_25_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_ENC_A1_PIN |
		ENCODER_ENC_A2_PIN |
		ENCODER_ENC_B1_PIN |
		ENCODER_ENC_B2_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER_ENC_A1_PIN |
		ENCODER_ENC_A2_PIN |
		ENCODER_ENC_B1_PIN |
		ENCODER_ENC_B2_PIN);
    DL_GPIO_clearPins(GPIOB, MOTOR_DIR_L_DIR1_PIN |
		MOTOR_DIR_L_DIR2_PIN |
		MOTOR_DIR_R_DIR2_PIN |
		GPIO_POOL_GPIO_PB1_PIN |
		GPIO_POOL_GPIO_PB10_PIN |
		GPIO_POOL_GPIO_PB11_PIN |
		GPIO_POOL_GPIO_PB14_PIN);
    DL_GPIO_enableOutput(GPIOB, MOTOR_DIR_L_DIR1_PIN |
		MOTOR_DIR_L_DIR2_PIN |
		MOTOR_DIR_R_DIR2_PIN |
		GPIO_POOL_GPIO_PB1_PIN |
		GPIO_POOL_GPIO_PB10_PIN |
		GPIO_POOL_GPIO_PB11_PIN |
		GPIO_POOL_GPIO_PB14_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_MOTORClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_MOTORConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 4000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_MOTOR_init(void) {

    DL_TimerG_setClockConfig(
        PWM_MOTOR_INST, (DL_TimerG_ClockConfig *) &gPWM_MOTORClockConfig);

    DL_TimerG_initPWMMode(
        PWM_MOTOR_INST, (DL_TimerG_PWMConfig *) &gPWM_MOTORConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_MOTOR_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 2000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 2000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_MOTOR_INST);


    
    DL_TimerG_setCCPDirection(PWM_MOTOR_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   125000 Hz = 32000000 Hz / (1 * (255 + 1))
 */
static const DL_TimerG_ClockConfig gCTRL_TIMERClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 255U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CTRL_TIMER_INST_LOAD_VALUE = (5ms * 125000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gCTRL_TIMERTimerConfig = {
    .period     = CTRL_TIMER_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_CTRL_TIMER_init(void) {

    DL_TimerG_setClockConfig(CTRL_TIMER_INST,
        (DL_TimerG_ClockConfig *) &gCTRL_TIMERClockConfig);

    DL_TimerG_initTimerMode(CTRL_TIMER_INST,
        (DL_TimerG_TimerConfig *) &gCTRL_TIMERTimerConfig);
    DL_TimerG_enableInterrupt(CTRL_TIMER_INST , DL_TIMERG_INTERRUPT_LOAD_EVENT);
    DL_TimerG_enableClock(CTRL_TIMER_INST);





}


static const DL_I2C_ClockConfig gI2C_BUSClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_BUS_init(void) {

    DL_I2C_setClockConfig(I2C_BUS_INST,
        (DL_I2C_ClockConfig *) &gI2C_BUSClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_BUS_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_BUS_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_BUS_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_BUS_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(I2C_BUS_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_BUS_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_BUS_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_BUS_INST);


}

static const DL_UART_Main_ClockConfig gUART_CONSOLEClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_CONSOLEConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_CONSOLE_init(void)
{
    DL_UART_Main_setClockConfig(UART_CONSOLE_INST, (DL_UART_Main_ClockConfig *) &gUART_CONSOLEClockConfig);

    DL_UART_Main_init(UART_CONSOLE_INST, (DL_UART_Main_Config *) &gUART_CONSOLEConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_CONSOLE_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_CONSOLE_INST, UART_CONSOLE_IBRD_32_MHZ_115200_BAUD, UART_CONSOLE_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_CONSOLE_INST);
}
static const DL_UART_Main_ClockConfig gUART_EXTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_EXTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_EXT_init(void)
{
    DL_UART_Main_setClockConfig(UART_EXT_INST, (DL_UART_Main_ClockConfig *) &gUART_EXTClockConfig);

    DL_UART_Main_init(UART_EXT_INST, (DL_UART_Main_Config *) &gUART_EXTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_EXT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_EXT_INST, UART_EXT_IBRD_32_MHZ_115200_BAUD, UART_EXT_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_EXT_INST);
}
static const DL_UART_Main_ClockConfig gUART_PBClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_PBConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_PB_init(void)
{
    DL_UART_Main_setClockConfig(UART_PB_INST, (DL_UART_Main_ClockConfig *) &gUART_PBClockConfig);

    DL_UART_Main_init(UART_PB_INST, (DL_UART_Main_Config *) &gUART_PBConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_PB_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_PB_INST, UART_PB_IBRD_32_MHZ_115200_BAUD, UART_PB_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_PB_INST);
}

