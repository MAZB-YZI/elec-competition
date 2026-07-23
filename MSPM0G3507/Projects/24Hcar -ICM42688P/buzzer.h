#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdbool.h>
#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Beep(uint32_t duration_ms);
void Buzzer_Update5ms(void);
void Buzzer_Stop(void);

#endif /* __BUZZER_H__ */
