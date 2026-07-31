/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG0
#define PWM_MOTOR_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for CTRL_TIMER */
#define CTRL_TIMER_INST                                                  (TIMG6)
#define CTRL_TIMER_INST_IRQHandler                              TIMG6_IRQHandler
#define CTRL_TIMER_INST_INT_IRQN                                (TIMG6_INT_IRQn)
#define CTRL_TIMER_INST_LOAD_VALUE                                        (624U)



/* Defines for UART_CONSOLE */
#define UART_CONSOLE_INST                                                  UART2
#define UART_CONSOLE_INST_FREQUENCY                                     32000000
#define UART_CONSOLE_INST_IRQHandler                            UART2_IRQHandler
#define UART_CONSOLE_INST_INT_IRQN                                UART2_INT_IRQn
#define GPIO_UART_CONSOLE_RX_PORT                                          GPIOA
#define GPIO_UART_CONSOLE_TX_PORT                                          GPIOA
#define GPIO_UART_CONSOLE_RX_PIN                                  DL_GPIO_PIN_24
#define GPIO_UART_CONSOLE_TX_PIN                                  DL_GPIO_PIN_23
#define GPIO_UART_CONSOLE_IOMUX_RX                               (IOMUX_PINCM54)
#define GPIO_UART_CONSOLE_IOMUX_TX                               (IOMUX_PINCM53)
#define GPIO_UART_CONSOLE_IOMUX_RX_FUNC                IOMUX_PINCM54_PF_UART2_RX
#define GPIO_UART_CONSOLE_IOMUX_TX_FUNC                IOMUX_PINCM53_PF_UART2_TX
#define UART_CONSOLE_BAUD_RATE                                          (115200)
#define UART_CONSOLE_IBRD_32_MHZ_115200_BAUD                                (17)
#define UART_CONSOLE_FBRD_32_MHZ_115200_BAUD                                (23)
/* Defines for UART_PB */
#define UART_PB_INST                                                       UART3
#define UART_PB_INST_FREQUENCY                                          32000000
#define UART_PB_INST_IRQHandler                                 UART3_IRQHandler
#define UART_PB_INST_INT_IRQN                                     UART3_INT_IRQn
#define GPIO_UART_PB_RX_PORT                                               GPIOB
#define GPIO_UART_PB_TX_PORT                                               GPIOB
#define GPIO_UART_PB_RX_PIN                                        DL_GPIO_PIN_3
#define GPIO_UART_PB_TX_PIN                                        DL_GPIO_PIN_2
#define GPIO_UART_PB_IOMUX_RX                                    (IOMUX_PINCM16)
#define GPIO_UART_PB_IOMUX_TX                                    (IOMUX_PINCM15)
#define GPIO_UART_PB_IOMUX_RX_FUNC                     IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_PB_IOMUX_TX_FUNC                     IOMUX_PINCM15_PF_UART3_TX
#define UART_PB_BAUD_RATE                                                (38400)
#define UART_PB_IBRD_32_MHZ_38400_BAUD                                      (52)
#define UART_PB_FBRD_32_MHZ_38400_BAUD                                       (5)
/* Defines for JY61P_UART */
#define JY61P_UART_INST                                                    UART0
#define JY61P_UART_INST_FREQUENCY                                       32000000
#define JY61P_UART_INST_IRQHandler                              UART0_IRQHandler
#define JY61P_UART_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_JY61P_UART_RX_PORT                                            GPIOA
#define GPIO_JY61P_UART_TX_PORT                                            GPIOA
#define GPIO_JY61P_UART_RX_PIN                                     DL_GPIO_PIN_1
#define GPIO_JY61P_UART_TX_PIN                                     DL_GPIO_PIN_0
#define GPIO_JY61P_UART_IOMUX_RX                                  (IOMUX_PINCM2)
#define GPIO_JY61P_UART_IOMUX_TX                                  (IOMUX_PINCM1)
#define GPIO_JY61P_UART_IOMUX_RX_FUNC                   IOMUX_PINCM2_PF_UART0_RX
#define GPIO_JY61P_UART_IOMUX_TX_FUNC                   IOMUX_PINCM1_PF_UART0_TX
#define JY61P_UART_BAUD_RATE                                              (9600)
#define JY61P_UART_IBRD_32_MHZ_9600_BAUD                                   (208)
#define JY61P_UART_FBRD_32_MHZ_9600_BAUD                                    (21)





/* Defines for ir_sensor */
#define ir_sensor_INST                                                      ADC1
#define ir_sensor_INST_IRQHandler                                ADC1_IRQHandler
#define ir_sensor_INST_INT_IRQN                                  (ADC1_INT_IRQn)
#define ir_sensor_ADCMEM_0                                    DL_ADC12_MEM_IDX_0
#define ir_sensor_ADCMEM_0_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ir_sensor_ADCMEM_0_REF_VOLTAGE_V                                     3.3
#define GPIO_ir_sensor_C0_PORT                                             GPIOA
#define GPIO_ir_sensor_C0_PIN                                     DL_GPIO_PIN_15
#define GPIO_ir_sensor_IOMUX_C0                                  (IOMUX_PINCM37)
#define GPIO_ir_sensor_IOMUX_C0_FUNC              (IOMUX_PINCM37_PF_UNCONNECTED)



/* Port definition for Pin Group BUZZER */
#define BUZZER_PORT                                                      (GPIOA)

/* Defines for BUZZER_PIN: GPIOA.7 with pinCMx 14 on package pin 49 */
#define BUZZER_BUZZER_PIN_PIN                                    (DL_GPIO_PIN_7)
#define BUZZER_BUZZER_PIN_IOMUX                                  (IOMUX_PINCM14)
/* Port definition for Pin Group GRAY_SENSOR */
#define GRAY_SENSOR_PORT                                                 (GPIOB)

/* Defines for GRAY_CLK: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GRAY_SENSOR_GRAY_CLK_PIN                                 (DL_GPIO_PIN_6)
#define GRAY_SENSOR_GRAY_CLK_IOMUX                               (IOMUX_PINCM23)
/* Defines for GRAY_DAT: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GRAY_SENSOR_GRAY_DAT_PIN                                 (DL_GPIO_PIN_7)
#define GRAY_SENSOR_GRAY_DAT_IOMUX                               (IOMUX_PINCM24)
/* Defines for L_DIR1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define MOTOR_DIR_L_DIR1_PORT                                            (GPIOB)
#define MOTOR_DIR_L_DIR1_PIN                                    (DL_GPIO_PIN_19)
#define MOTOR_DIR_L_DIR1_IOMUX                                   (IOMUX_PINCM45)
/* Defines for L_DIR2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define MOTOR_DIR_L_DIR2_PORT                                            (GPIOB)
#define MOTOR_DIR_L_DIR2_PIN                                    (DL_GPIO_PIN_17)
#define MOTOR_DIR_L_DIR2_IOMUX                                   (IOMUX_PINCM43)
/* Defines for R_DIR1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define MOTOR_DIR_R_DIR1_PORT                                            (GPIOA)
#define MOTOR_DIR_R_DIR1_PIN                                    (DL_GPIO_PIN_16)
#define MOTOR_DIR_R_DIR1_IOMUX                                   (IOMUX_PINCM38)
/* Defines for R_DIR2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define MOTOR_DIR_R_DIR2_PORT                                            (GPIOB)
#define MOTOR_DIR_R_DIR2_PIN                                    (DL_GPIO_PIN_24)
#define MOTOR_DIR_R_DIR2_IOMUX                                   (IOMUX_PINCM52)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for GPIO_PB1: GPIOB.1 with pinCMx 13 on package pin 48 */
// pins affected by this interrupt request:["GPIO_PB1","GPIO_PB10","GPIO_PB11","GPIO_PB14"]
#define KEY_INT_IRQN                                            (GPIOB_INT_IRQn)
#define KEY_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define KEY_GPIO_PB1_IIDX                                    (DL_GPIO_IIDX_DIO1)
#define KEY_GPIO_PB1_PIN                                         (DL_GPIO_PIN_1)
#define KEY_GPIO_PB1_IOMUX                                       (IOMUX_PINCM13)
/* Defines for GPIO_PB10: GPIOB.10 with pinCMx 27 on package pin 62 */
#define KEY_GPIO_PB10_IIDX                                  (DL_GPIO_IIDX_DIO10)
#define KEY_GPIO_PB10_PIN                                       (DL_GPIO_PIN_10)
#define KEY_GPIO_PB10_IOMUX                                      (IOMUX_PINCM27)
/* Defines for GPIO_PB11: GPIOB.11 with pinCMx 28 on package pin 63 */
#define KEY_GPIO_PB11_IIDX                                  (DL_GPIO_IIDX_DIO11)
#define KEY_GPIO_PB11_PIN                                       (DL_GPIO_PIN_11)
#define KEY_GPIO_PB11_IOMUX                                      (IOMUX_PINCM28)
/* Defines for GPIO_PB14: GPIOB.14 with pinCMx 31 on package pin 2 */
#define KEY_GPIO_PB14_IIDX                                  (DL_GPIO_IIDX_DIO14)
#define KEY_GPIO_PB14_PIN                                       (DL_GPIO_PIN_14)
#define KEY_GPIO_PB14_IOMUX                                      (IOMUX_PINCM31)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOA)

/* Defines for ENC_A1: GPIOA.27 with pinCMx 60 on package pin 31 */
// pins affected by this interrupt request:["ENC_A1","ENC_A2","ENC_B1","ENC_B2"]
#define ENCODER_INT_IRQN                                        (GPIOA_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODER_ENC_A1_IIDX                                 (DL_GPIO_IIDX_DIO27)
#define ENCODER_ENC_A1_PIN                                      (DL_GPIO_PIN_27)
#define ENCODER_ENC_A1_IOMUX                                     (IOMUX_PINCM60)
/* Defines for ENC_A2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define ENCODER_ENC_A2_IIDX                                 (DL_GPIO_IIDX_DIO26)
#define ENCODER_ENC_A2_PIN                                      (DL_GPIO_PIN_26)
#define ENCODER_ENC_A2_IOMUX                                     (IOMUX_PINCM59)
/* Defines for ENC_B1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define ENCODER_ENC_B1_IIDX                                 (DL_GPIO_IIDX_DIO14)
#define ENCODER_ENC_B1_PIN                                      (DL_GPIO_PIN_14)
#define ENCODER_ENC_B1_IOMUX                                     (IOMUX_PINCM36)
/* Defines for ENC_B2: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODER_ENC_B2_IIDX                                 (DL_GPIO_IIDX_DIO25)
#define ENCODER_ENC_B2_PIN                                      (DL_GPIO_PIN_25)
#define ENCODER_ENC_B2_IOMUX                                     (IOMUX_PINCM55)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_CTRL_TIMER_init(void);
void SYSCFG_DL_UART_CONSOLE_init(void);
void SYSCFG_DL_UART_PB_init(void);
void SYSCFG_DL_JY61P_UART_init(void);
void SYSCFG_DL_ir_sensor_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
