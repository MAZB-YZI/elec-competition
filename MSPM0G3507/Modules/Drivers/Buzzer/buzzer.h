#ifndef MODULES_DRIVERS_BUZZER_H
#define MODULES_DRIVERS_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Beep(uint32_t duration_ms);
void Buzzer_Update1ms(void);
void Buzzer_Stop(void);

/* Board hook: provide buzzer output on the active board. */
void BuzzerHal_SetOutput(bool on);

#endif /* MODULES_DRIVERS_BUZZER_H */
