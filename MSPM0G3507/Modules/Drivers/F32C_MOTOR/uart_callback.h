#ifndef _UART_CALLBACK_H_
#define _UART_CALLBACK_H_
#include "board.h"

extern int Motor1_Speed, Motor2_Speed;
extern int motor1_Current_Speed, motor2_Current_Speed;
extern int Motor1_T_Position, Motor2_T_Position;
extern int Motor1_Current_Position, Motor2_Current_Position;
extern volatile uint8_t motor1_position_valid;
extern volatile uint8_t motor2_position_valid;

void usart1_send(u8 data);
void USART1_SEND(u8 *data, u8 len);
void uart1_send_data(uint8_t data);
void uart1_send_SendArray(uint8_t *data, uint8_t len);
u8 BCC_Sum1(u8 *usart_data, unsigned char Count_Number);

#endif