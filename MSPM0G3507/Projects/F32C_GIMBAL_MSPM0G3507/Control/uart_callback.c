#include "ti_msp_dl_config.h"
#include "board.h"
#include "uart_callback.h"

/* UART1 ���ջ��棺�������֡�̶�Ϊ 9 �ֽڡ� */
u8 usart1_receive_data[10];

/* UART1 ���� 1 �ֽڣ��ȴ����� FIFO �п�λ��д�롣 */
void usart1_send(u8 data)
{
    while (DL_UART_isTXFIFOFull(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, data);
}

/* UART1 �������飬���ֲο����� USARTx_SEND �ĵ��÷�� */
void USART1_SEND(u8 *data, u8 len)
{
    u8 i = 0;

    for (i = 0; i < len; i++)
    {
        usart1_send(data[i]);
    }
}

/* ���ݾɹ��������е� uart1_send_data ���ƣ�ʵ���Ե��òο����̷��ӿڡ� */
void uart1_send_data(uint8_t data)
{
    usart1_send(data);
}

/* ���ݾɹ��������е� uart1_send_SendArray ���ơ� */
void uart1_send_SendArray(uint8_t *data, uint8_t len)
{
    USART1_SEND(data, len);
}

/* BCC ���У�飺��֡ͷ��ʼ����ָ���ֽ������ֽ���� */
u8 BCC_Sum1(u8 *usart_data, unsigned char Count_Number)
{
    unsigned char crc_sum = 0, k;

    for (k = 0; k < Count_Number; k++)
    {
        crc_sum = crc_sum ^ usart_data[k];
    }

    return crc_sum;
}

/* UART1 �����жϣ����ֽ�ƴ�� 9 �ֽڷ���֡��У��ͨ�������ʵʱλ�á� */
void UART_1_INST_IRQHandler(void)
{
    u8 Usart1_Receive;
    static u8 Count = 0;

    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) == DL_UART_MAIN_IIDX_RX)
    {
        Usart1_Receive = DL_UART_Main_receiveData(UART_1_INST);

        /* �ȴ�֡ͷ 0x7A����������ֽڵ��½��մ�λ�� */
        if (Count == 0 && Usart1_Receive != 0x7A)
        {
            return;
        }

        usart1_receive_data[Count++] = Usart1_Receive;

        if (Count >= 9)
        {
            Count = 0;

            /* ֡β����Ϊ 0x7B�� */
            if (usart1_receive_data[8] != 0x7B)
            {
                return;
            }

            /* �� 8 �ֽ�Ϊǰ 7 �ֽ� BCC У�顣 */
            if (usart1_receive_data[7] == BCC_Sum1(usart1_receive_data, 7))
            {
                /* ��ַ 0x01���������� 0x01��1 �ŵ��ʵʱλ�á� */
                if (usart1_receive_data[1] == 0x01)
                {
                    if (usart1_receive_data[2] == 0x01)
                    {
                        Motor1_Current_Position = ((usart1_receive_data[3] << 24) +
                                                  (usart1_receive_data[4] << 16) +
                                                  (usart1_receive_data[5] << 8) +
                                                   usart1_receive_data[6]) / 10;
                    }
                    else if (usart1_receive_data[2] == 0x00)
                    {
                        motor1_Current_Speed = (usart1_receive_data[3] << 24) +
                                               (usart1_receive_data[4] << 16) +
                                               (usart1_receive_data[5] << 8) +
                                                usart1_receive_data[6];
                        motor1_position_valid = 1;
                    }
                }

                /* ��ַ 0x02���������� 0x01��2 �ŵ��ʵʱλ�á� */
                if (usart1_receive_data[1] == 0x02)
                {
                    if (usart1_receive_data[2] == 0x01)
                    {
                        Motor2_Current_Position = ((usart1_receive_data[3] << 24) +
                                                  (usart1_receive_data[4] << 16) +
                                                  (usart1_receive_data[5] << 8) +
                                                   usart1_receive_data[6]) / 10;
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
