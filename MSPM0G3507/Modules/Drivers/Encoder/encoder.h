#ifndef MODULES_DRIVERS_ENCODER_H
#define MODULES_DRIVERS_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t left;
    int32_t right;
} EncoderCounts;

typedef struct {
    float left;
    float right;
} EncoderPair;

void Encoder_Init(void);
void Encoder_SetDirection(int left_dir, int right_dir);  /* +1或-1，前进时速度>0 */
void Encoder_OnLeftAEdge(void);
void Encoder_OnRightAEdge(void);
void Encoder_Update(float dt_s);
EncoderCounts Encoder_GetCount(void);
EncoderPair Encoder_GetSpeed(void);
EncoderPair Encoder_GetDistance(void);
void Encoder_ResetDistance(void);

/* Board hook: provide encoder channel-B sampling from the active board. */
bool EncoderHal_ReadPhaseB(bool left);

#endif /* MODULES_DRIVERS_ENCODER_H */
