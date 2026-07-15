#include "key.h"

#include "ti_msp_dl_config.h"

/*
 * PB21 按键稳定扫描版
 * - 按下接地，松开上拉
 * - 每次单击只返回一次 USEKEY_single_click
 * - 适合主循环里 10ms 左右轮询
 */

uint8_t keyValue(void)
{
    return (DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) & KEY_key_PIN) ? 0U : 1U;
}

UserKeyState_t user_key_scan(uint16_t freq)
{
    static uint16_t down_ticks = 0U;
    static uint16_t up_ticks = 0U;
    static uint8_t  pressed = 0U;
    static uint8_t  latched = 0U;

    if (freq == 0U) {
        return USEKEY_stateless;
    }

    /*
     * 周期换算：freq = 100 表示 10ms 扫一次
     * 这里用简化去抖：
     * - 按下连续稳定 20ms 才认为按下
     * - 松开连续稳定 20ms 才允许下一次触发
     */
    if (keyValue() == 1U) {
        if (down_ticks < 0xFFFFU) {
            ++down_ticks;
        }
        up_ticks = 0U;
    } else {
        if (up_ticks < 0xFFFFU) {
            ++up_ticks;
        }
        down_ticks = 0U;
    }

    if (pressed == 0U) {
        if (down_ticks >= (uint16_t)(freq / 50U)) {
            pressed = 1U;
            latched = 1U;
            return USEKEY_single_click;
        }
    } else {
        if (up_ticks >= (uint16_t)(freq / 50U)) {
            pressed = 0U;
            latched = 0U;
        }
    }

    if (latched == 1U) {
        return USEKEY_stateless;
    }

    return USEKEY_stateless;
}
