#include "buzzer.h"
#include "hcar_hal.h"
static volatile uint32_t remaining;
void Buzzer_Init(void) { Buzzer_Stop(); }
void Buzzer_Beep(uint32_t duration_ms) { remaining=duration_ms; HCarHal_SetBuzzer(duration_ms != 0); }
void Buzzer_Update1ms(void) { if (remaining && --remaining == 0) HCarHal_SetBuzzer(false); }
void Buzzer_Stop(void) { remaining=0; HCarHal_SetBuzzer(false); }
