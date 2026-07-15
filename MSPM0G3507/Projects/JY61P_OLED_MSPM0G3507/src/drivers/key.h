#ifndef KEY_H
#define KEY_H

#include <stdint.h>

typedef enum {
    USEKEY_stateless = 0,
    USEKEY_single_click,
    USEKEY_double_click,
    USEKEY_long_click
} UserKeyState_t;

UserKeyState_t user_key_scan(uint16_t freq);
uint8_t keyValue(void);

#endif /* KEY_H */
