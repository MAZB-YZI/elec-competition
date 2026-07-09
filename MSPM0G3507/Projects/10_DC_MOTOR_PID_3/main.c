/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"

int status = 0;
extern float target_speed_1;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    OLED_Clear();
    // NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
    NVIC_EnableIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
    DL_ADC12_enableConversions(xuanniu_INST);
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50, GPIO_SERVO_C1_IDX);
    motor_init(1);
    // motor_set_duty(1, 2000);
    target_speed_1 = 300;
    

    while (1) {
        delay_ms(1000);
        motor_set_direction(1, 1);
        delay_ms(1000);
        motor_set_direction(1, 1);
        

        
        
        
        
        // // 通知ADC开始采样
        // DL_ADC12_startConversion(xuanniu_INST);

        // //等Adc采样完
        // delay_ms(10);

        // // 获取ADC采样结果
        // uint16_t adc_result = DL_ADC12_getMemResult(xuanniu_INST, xuanniu_ADCMEM_0);
        // float_t adc_value = adc_result * xuanniu_ADCMEM_0_REF_VOLTAGE_V / 4096.0; // Assuming 12-bit ADC resolution
        
        // char oled_str[50];
        // sprintf(oled_str, "ADC: %.2f V", adc_value);
        // OLED_ShowString(0, 32, (u8 *)oled_str, 16);
        // OLED_Refresh();
        

        // if(status == 0){
        //     OLED_Clear();
        //     OLED_ShowString(0, 0, (u8 *)"status: 0", 16);
        //     OLED_Refresh();
        // } 
        // else if(status == 1){
        //     OLED_Clear();
        //     OLED_ShowString(0, 0, (u8 *)"status: 1", 16);
        //     OLED_Refresh();
        // }
        // else if(status == 2){
        //     OLED_Clear();
        //     OLED_ShowString(0, 0, (u8 *)"status: 2", 16);
        //     OLED_Refresh();
        // }
        

        // Toggle the LED every 500 ms
        // char oled_str[50];
        // int int_a = 20;
        // sprintf(oled_str, "Integer: %d", int_a);
        // OLED_ShowString(0, 46, (u8 *)oled_str, 16);
        // OLED_Refresh();
        

        // OLED_ShowString(0, 0, (u8 *)"Hello, TI!", 16);
        // OLED_Refresh();
        // delay_ms(500);
        // DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        // DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
        // delay_ms(500);
        // DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
        // DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
        // UART_send_string(PRINT_INST, "hello, ti!\n");
    }
}
