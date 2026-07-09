#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
char txbuff[64];
void app_entry(void){
	uint32_t count=0;
	while(1)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		sprintf(txbuff, "HNDX_马梓博 count=%d\n", count++);
		HAL_UART_Transmit(&huart1,(uint8_t *)txbuff,strlen(txbuff),0xFFFFFFFF);
		HAL_Delay(500);
	}
}