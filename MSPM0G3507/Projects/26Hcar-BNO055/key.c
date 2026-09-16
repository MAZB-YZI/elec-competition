/**
 * key.c — 4 按键驱动（参考队友 d285bc2 的稳定状态消抖算法）
 *
 * 硬件：按键一端接 GPIO，另一端接 GND
 * 配置：输入上拉，按下=低电平，松开=高电平
 * 扫描：5ms 定时器中断中调用 Key_Scan5ms()
 * 查询：主循环调用 Key_GetPressEvent() 获取一次性按下事件
 */

#include "key.h"
#include "ti_msp_dl_config.h"

#define KEY_DEBOUNCE_TICKS  4U   /* 4×5ms = 20ms 消抖 */

/* 引脚表 */
static GPIO_Regs *const g_key_port[KEY_ID_NUM] = {
    GPIOB, GPIOB, GPIOB, GPIOB
};
static const uint32_t g_key_pin[KEY_ID_NUM] = {
    DL_GPIO_PIN_1,   /* K1 = PB1 */
    DL_GPIO_PIN_10,  /* K2 = PB10 */
    DL_GPIO_PIN_11,  /* K3 = PB11 */
    DL_GPIO_PIN_14   /* K4 = PB14 */
};
static const uint32_t g_key_iomux[KEY_ID_NUM] = {
    IOMUX_PINCM13,   /* PB1 */
    IOMUX_PINCM27,   /* PB10 */
    IOMUX_PINCM28,   /* PB11 */
    IOMUX_PINCM31    /* PB14 */
};

/* 状态 */
static uint8_t g_key_cnt[KEY_ID_NUM];
static bool g_key_state[KEY_ID_NUM];
static volatile bool g_key_press_event[KEY_ID_NUM];

/* ================================================================
 *  初始化：配置为输入上拉，关闭输出使能
 * ================================================================ */
void Key_Init(void)
{
    for (uint8_t i = 0; i < KEY_ID_NUM; i++) {
        DL_GPIO_disableOutput(g_key_port[i], g_key_pin[i]);
        DL_GPIO_initDigitalInputFeatures(
            g_key_iomux[i], DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP_DISABLE);
        g_key_port[i]->DOECLR31_0 = g_key_pin[i];

        g_key_cnt[i] = 0U;
        g_key_state[i] = (DL_GPIO_readPins(g_key_port[i], g_key_pin[i]) == 0U);
        g_key_press_event[i] = false;
    }
}

/* ================================================================
 *  扫描：每 5ms 由定时器中断调用
 *  参考队友的稳定状态消抖：按下和松开都消抖
 * ================================================================ */
void Key_Scan5ms(void)
{
    for (uint8_t i = 0; i < KEY_ID_NUM; i++) {
        bool raw_pressed = (DL_GPIO_readPins(g_key_port[i], g_key_pin[i]) == 0U);

        if (raw_pressed == g_key_state[i]) {
            /* 状态未变，计数归零 */
            g_key_cnt[i] = 0U;
        } else if (++g_key_cnt[i] >= KEY_DEBOUNCE_TICKS) {
            /* 连续 N 次不同，确认状态变化 */
            g_key_cnt[i] = 0U;
            g_key_state[i] = raw_pressed;

            /* 只在按下瞬间产生事件 */
            if (raw_pressed) {
                g_key_press_event[i] = true;
            }
        }
    }
}

/* ================================================================
 *  查询：获取按下事件（一次性，读后清除）
 * ================================================================ */
bool Key_GetPressEvent(KeyId_t id)
{
    if (id >= KEY_ID_NUM) return false;

    __disable_irq();
    bool event = g_key_press_event[id];
    g_key_press_event[id] = false;
    __enable_irq();

    return event;
}
