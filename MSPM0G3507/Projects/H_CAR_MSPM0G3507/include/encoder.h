#ifndef HCAR_ENCODER_H
#define HCAR_ENCODER_H
#include <stdint.h>
typedef struct { int32_t left; int32_t right; } EncoderCounts;
typedef struct { float left; float right; } EncoderPair;
void Encoder_Init(void);
void Encoder_OnLeftAEdge(void);
void Encoder_OnRightAEdge(void);
void Encoder_Update(float dt_s);
EncoderCounts Encoder_GetCount(void);
EncoderPair Encoder_GetSpeed(void);
EncoderPair Encoder_GetDistance(void);
void Encoder_ResetDistance(void);
#endif
