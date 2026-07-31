#ifndef __KEY_H__
#define __KEY_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
    KEY_ID_NUM
} KeyId_t;

void Key_Init(void);
void Key_GPIO_IRQHandler(void);
void Key_Scan5ms(void);
bool Key_GetPressEvent(KeyId_t id);

#endif /* __KEY_H__ */
