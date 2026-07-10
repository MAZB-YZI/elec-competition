#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"

typedef enum {
	USEKEY_stateless,
	USEKEY_single_click,
	USEKEY_double_click,
	USEKEY_long_click
}UserKeyState_t;
UserKeyState_t key_scan(uint16_t freq);
UserKeyState_t user_key_scan(uint16_t freq);
uint8_t userKeyValue(void);
uint8_t keyValue(void);
u8 click(void);
#endif
