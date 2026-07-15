#include "ti_msp_dl_config.h"
#include "board.h"
#include "uart_callback.h"

/* UART1 接收缓冲区：电机回复帧固定为 9 字节。 */
u8 usart1_receive_data[10];

/* UART1 发送 1 字节，等待发送 FIFO 有空位再写入。 */
void usart1_send(u8 data)
{
    while (DL_UART_isTXFIFOFull(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, data);
}

/* UART1 发送数组，参考官方 USARTx_SEND 的调用方式 */
void USART1_SEND(u8 *data, u8 len)
{
    u8 i = 0;

    for (i = 0; i < len; i++)
    {
        usart1_send(data[i]);
    }
}

/* 参考旧驱动中的 uart1_send_data 函数，实际可参考官方接口 */
void uart1_send_data(uint8_t data)
{
    usart1_send(data);
}

/* 参考旧驱动中的 uart1_send_SendArray 函数 */
void uart1_send_SendArray(uint8_t *data, uint8_t len)
{
    USART1_SEND(data, len);
}

/* BCC 异或校验：从帧头开始到指令字节前一字节止 */
u8 BCC_Sum1(u8 *usart_data, unsigned char Count_Number)
{
    unsigned char crc_sum = 0, k;

    for (k = 0; k < Count_Number; k++)
    {
        crc_sum = crc_sum ^ usart_data[k];
    }

    return crc_sum;
}

/* UART1 接收中断：逐字节拼出 9 字节回复帧，校验通过后解析实时位置。 */
void UART_1_INST_IRQHandler(void)
{
    u8 Usart1_Receive;
    static u8 Count = 0;

    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) == DL_UART_MAIN_IIDX_RX)
    {
        Usart1_Receive = DL_UART_Main_receiveData(UART_1_INST);

        /* 先找帧头 0x7A，否则其余字节的进入下一次接收位置。 */
        if (Count == 0 && Usart1_Receive != 0x7A)
        {
            return;
        }

        usart1_receive_data[Count++] = Usart1_Receive;

        if (Count >= 9)
        {
            Count = 0;

            /* 帧尾必须为 0x7B。 */
            if (usart1_receive_data[8] != 0x7B)
            {
                return;
            }

            /* 前 8 字节为前 7 字节 BCC 校验。 */
            if (usart1_receive_data[7] == BCC_Sum1(usart1_receive_data, 7))
            {
                /* 地址 0x01，指令类型 0x01：1 号电机的实时位置 */
                if (usart1_receive_data[1] == 0x01)
                {
                    if (usart1_receive_data[2] == 0x01)
                    {
                        Motor1_Current_Position = ((usart1_receive_data[3] << 24) +
                                                  (usart1_receive_data[4] << 16) +
                                                  (usart1_receive_data[5] << 8) +
                                                   usart1_receive_data[6]) / 10;
                        motor1_position_valid = 1;  /* 收到位置信息，设置有效标志 */
                    }
                    else if (usart1_receive_data[2] == 0x00)
                    {
                        motor1_Current_Speed = (usart1_receive_data[3] << 24) +
                                               (usart1_receive_data[4] << 16) +
                                               (usart1_receive_data[5] << 8) +
                                                usart1_receive_data[6];
                    }
                }

                /* 地址 0x02，指令类型 0x01：2 号电机的实时位置 */
                if (usart1_receive_data[1] == 0x02)
                {
                    if (usart1_receive_data[2] == 0x01)
                    {
                        Motor2_Current_Position = ((usart1_receive_data[3] << 24) +
                                                  (usart1_receive_data[4] << 16) +
                                                  (usart1_receive_data[5] << 8) +
                                                   usart1_receive_data[6]) / 10;
                        motor2_position_valid = 1;  /* 收到位置信息，设置有效标志 */
                    }
                    else if (usart1_receive_data[2] == 0x00)
                    {
                        motor2_Current_Speed = (usart1_receive_data[3] << 24) +
                                               (usart1_receive_data[4] << 16) +
                                               (usart1_receive_data[5] << 8) +
                                                usart1_receive_data[6];
                    }
                }
            }
        }
    }
}
