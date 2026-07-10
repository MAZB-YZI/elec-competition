#include "key.h"

uint8_t keyValue(void)
{
    return ( DL_GPIO_readPins(KEY_PORT,KEY_key_PIN) & KEY_key_PIN ) > 0 ? 0:1;
}
// 参考例程 click()：检测一次按下沿，按住不重复触发。
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
    return (DL_GPIO_readPins(USER_KEY_PORT, USER_KEY_PIN_0_PIN) & USER_KEY_PIN_0_PIN) > 0 ? 0 : 1;
}


UserKeyState_t key_scan(uint16_t freq)
{
    static uint16_t time_core;//走时核心
    static uint16_t long_press_time;//长按识别
    static uint8_t press_flag=0;//按键按下标记
    static uint8_t check_once=0;//是否已经识别1次标记
	
    float Count_time = (((float)(1.0f/(float)freq))*1000.0f);//算出计1需要多少个毫秒

    if(check_once)//完成了识别，则清空所有变量
    {
        press_flag=0;//完成了1次识别，标记清零
        time_core=0;//完成了1次识别，时间清零
        long_press_time=0;//完成了1次识别，时间清零
    }
    if(check_once&&1 == keyValue()) check_once=0; //完成扫描后按键被弹起，则开启下一次扫描

    if(0==keyValue()&&check_once==0)//按键扫描
    {
        press_flag=1;//标记被按下1次
        long_press_time++;		  
    }

    if(long_press_time>(uint16_t)(500.0f/Count_time))// 长按1秒
    {	
        check_once=1;//标记已被识别
        return USEKEY_long_click; //长按
    }

    //按键被按下1次又弹起后，开启内核走时
    if(press_flag&&1==keyValue())
    {
        time_core++; 
    }		
	
    if(press_flag&&(time_core>(uint16_t)(50.0f/Count_time)&&time_core<(uint16_t)(300.0f/Count_time)))//50~700ms内被再次按下
    {
        if(0==keyValue()) //如果再次按下
        {
            check_once=1;//标记已被识别
            return USEKEY_double_click; //标记为双击
        }
    }
    else if(press_flag&&time_core>(uint16_t)(300.0f/Count_time))
    {
        check_once=1;//标记已被识别
        return USEKEY_single_click; //800ms后还没被按下，则是单击
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
    if (check_once && 0 == userKeyValue()) check_once = 0;  // 松开=0

    if (1 == userKeyValue() && check_once == 0)             // 按下=1
    {
        press_flag = 1;
        long_press_time++;
    }
    if (long_press_time > (uint16_t)(500.0f / Count_time))
    {
        check_once = 1;
        return USEKEY_long_click;
    }
    if (press_flag && 0 == userKeyValue())                  // 松开=0
    {
        time_core++;
    }
    if (press_flag && (time_core > (uint16_t)(50.0f / Count_time) &&
                       time_core < (uint16_t)(300.0f / Count_time)))
    {
        if (1 == userKeyValue())                            // 再次按下=1
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

