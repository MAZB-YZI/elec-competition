#ifndef __KEY_H__
#define __KEY_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    KEY_ID_K1 = 0,   /* PB1  - 上一题 */
    KEY_ID_K2,        /* PB10 - 下一题 */
    KEY_ID_K3,        /* PB11 - START */
    KEY_ID_K4,        /* PB14 - STOP */
    KEY_ID_NUM
} KeyId_t;

void Key_Init(void);
void Key_Scan5ms(void);
bool Key_GetPressEvent(KeyId_t id);

#endif /* __KEY_H__ */
