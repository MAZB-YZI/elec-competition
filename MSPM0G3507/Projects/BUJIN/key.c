#include "key.h"
#include "ti_msp_dl_config.h"

#define KEY_DEBOUNCE_TICKS 4U

static uint8_t g_key_cnt[KEY_ID_NUM];
static bool g_key_state[KEY_ID_NUM];
static volatile bool g_key_press_event[KEY_ID_NUM];
static volatile bool g_key_irq_pending;

static bool Key_ReadRawPressed(KeyId_t id)
{
    uint32_t pin = 0U;

    switch (id) {
    case KEY_ID_1:
        pin = KEY_GPIO_PB1_PIN;
        break;
    case KEY_ID_2:
        pin = KEY_GPIO_PB10_PIN;
        break;
    case KEY_ID_3:
        pin = KEY_GPIO_PB11_PIN;
        break;
    case KEY_ID_4:
        pin = KEY_GPIO_PB14_PIN;
        break;
    default:
        return false;
    }

    return (DL_GPIO_readPins(KEY_PORT, pin) == 0U);
}

void Key_Init(void)
{
    for (uint8_t i = 0; i < KEY_ID_NUM; i++) {
        g_key_cnt[i] = 0U;
        g_key_state[i] = Key_ReadRawPressed((KeyId_t)i);
        g_key_press_event[i] = false;
    }

    g_key_irq_pending = false;
    DL_GPIO_setLowerPinsPolarity(KEY_PORT,
        DL_GPIO_PIN_1_EDGE_FALL | DL_GPIO_PIN_10_EDGE_FALL |
        DL_GPIO_PIN_11_EDGE_FALL | DL_GPIO_PIN_14_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(KEY_PORT,
        KEY_GPIO_PB1_PIN | KEY_GPIO_PB10_PIN |
        KEY_GPIO_PB11_PIN | KEY_GPIO_PB14_PIN);
    NVIC_ClearPendingIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(KEY_INT_IRQN);
}

void Key_GPIO_IRQHandler(void)
{
    DL_GPIO_IIDX iidx = DL_GPIO_getPendingInterrupt(KEY_PORT);

    if ((iidx == KEY_GPIO_PB1_IIDX) ||
        (iidx == KEY_GPIO_PB10_IIDX) ||
        (iidx == KEY_GPIO_PB11_IIDX) ||
        (iidx == KEY_GPIO_PB14_IIDX)) {
        g_key_irq_pending = true;
        DL_GPIO_clearInterruptStatus(KEY_PORT,
            KEY_GPIO_PB1_PIN | KEY_GPIO_PB10_PIN |
            KEY_GPIO_PB11_PIN | KEY_GPIO_PB14_PIN);
    }
}

void Key_Scan5ms(void)
{
    (void)g_key_irq_pending;

    for (uint8_t i = 0; i < KEY_ID_NUM; i++) {
        bool raw_pressed = Key_ReadRawPressed((KeyId_t)i);

        if (raw_pressed == g_key_state[i]) {
            g_key_cnt[i] = 0U;
        } else if (++g_key_cnt[i] >= KEY_DEBOUNCE_TICKS) {
            g_key_cnt[i] = 0U;
            g_key_state[i] = raw_pressed;

            if (raw_pressed) {
                g_key_press_event[i] = true;
            }
        }
    }

    g_key_irq_pending = false;
}

bool Key_GetPressEvent(KeyId_t id)
{
    bool event;

    if (id >= KEY_ID_NUM) {
        return false;
    }

    __disable_irq();
    event = g_key_press_event[id];
    g_key_press_event[id] = false;
    __enable_irq();

    return event;
}
