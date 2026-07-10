#include "key.h"

extern int status;
uint32_t counter_1_A = 0;

uint8_t keyValue(void)
{
    return ( DL_GPIO_readPins(KEY_PORT,KEY_key_PIN) & KEY_key_PIN ) > 0 ? 0:1;
}
// �ο����� click()�����һ�ΰ����أ���ס���ظ�������
u8 click(void)
{
    static u8 flag_key = 1;

    if (flag_key && keyValue() == 1)
    {
        flag_key = 0;
        return 1;
    }
    else if (keyValue() == 0)
    {
        flag_key = 1;
    }

    return 0;
}
uint8_t userKeyValue(void)
{
    return (DL_GPIO_readPins(KEY_PORT, KEY_user_key_PIN) & KEY_user_key_PIN) > 0 ? 0 : 1;
}


UserKeyState_t key_scan(uint16_t freq)
{
    static uint16_t time_core;//��ʱ����
    static uint16_t long_press_time;//����ʶ��
    static uint8_t press_flag=0;//�������±��
    static uint8_t check_once=0;//�Ƿ��Ѿ�ʶ��1�α��
	
    float Count_time = (((float)(1.0f/(float)freq))*1000.0f);//�����1��Ҫ���ٸ�����

    if(check_once)//�����ʶ����������б���
    {
        press_flag=0;//�����1��ʶ�𣬱������
        time_core=0;//�����1��ʶ��ʱ������
        long_press_time=0;//�����1��ʶ��ʱ������
    }
    if(check_once&&1 == keyValue()) check_once=0; //���ɨ��󰴼�������������һ��ɨ��

    if(0==keyValue()&&check_once==0)//����ɨ��
    {
        press_flag=1;//��Ǳ�����1��
        long_press_time++;		  
    }

    if(long_press_time>(uint16_t)(500.0f/Count_time))// ����1��
    {	
        check_once=1;//����ѱ�ʶ��
        return USEKEY_long_click; //����
    }

    //����������1���ֵ���󣬿����ں���ʱ
    if(press_flag&&1==keyValue())
    {
        time_core++; 
    }		
	
    if(press_flag&&(time_core>(uint16_t)(50.0f/Count_time)&&time_core<(uint16_t)(300.0f/Count_time)))//50~700ms�ڱ��ٴΰ���
    {
        if(0==keyValue()) //����ٴΰ���
        {
            check_once=1;//����ѱ�ʶ��
            return USEKEY_double_click; //���Ϊ˫��
        }
    }
    else if(press_flag&&time_core>(uint16_t)(300.0f/Count_time))
    {
        check_once=1;//����ѱ�ʶ��
        return USEKEY_single_click; //800ms��û�����£����ǵ���
    }

    return USEKEY_stateless;
}
UserKeyState_t user_key_scan(uint16_t freq)
{
    static uint16_t time_core       = 0;
    static uint16_t long_press_time = 0;
    static uint8_t  press_flag      = 0;
    static uint8_t  check_once      = 0;

    float Count_time = (1.0f / (float)freq) * 1000.0f;

    if (check_once)
    {
        press_flag      = 0;
        time_core       = 0;
        long_press_time = 0;
    }
    if (check_once && 0 == userKeyValue()) check_once = 0;

    if (1 == userKeyValue() && check_once == 0)
    {
        press_flag = 1;
        long_press_time++;
    }
    if (long_press_time > (uint16_t)(500.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_long_click;
    }
    if (press_flag && 0 == userKeyValue())
    {
        time_core++;
    }
    if (press_flag && (time_core > (uint16_t)(50.0f / Count_time) &&
                       time_core < (uint16_t)(300.0f / Count_time)))
    {
        if (1 == userKeyValue())
        {
            check_once = 1;
            return USEKEY_double_click;
        }
    }
    else if (press_flag && time_core > (uint16_t)(300.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_single_click;
    }
    return USEKEY_stateless;
}

/* GPIO 中断处理（按键 + 编码器） */
void GROUP1_IRQHandler()
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_key_IIDX:
        status = (status + 1) % 3;
        break;
    case KEY_user_key_IIDX:
        status = (status + 3 - 1) % 3;
        break;
    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_AA_IIDX:
        counter_1_A++;
        break;
    default:
        break;
    }
}

