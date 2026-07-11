#include "encoder.h"
#include "hcar_hal.h"
#define ENCODER_COUNTS_PER_REV 260.0f
#define WHEEL_CIRCUMFERENCE_M 0.2104867f
static volatile int32_t left_count, right_count;
static int32_t last_left, last_right, zero_left, zero_right;
static EncoderPair speed;
static float count_to_m(int32_t count) { return count * WHEEL_CIRCUMFERENCE_M / ENCODER_COUNTS_PER_REV; }
void Encoder_Init(void) { left_count=right_count=last_left=last_right=zero_left=zero_right=0; speed.left=speed.right=0; }
void Encoder_OnLeftAEdge(void) { left_count += HCarHal_ReadEncoderB(true) ? -1 : 1; }
void Encoder_OnRightAEdge(void) { right_count += HCarHal_ReadEncoderB(false) ? -1 : 1; }
void Encoder_Update(float dt_s) { int32_t l=left_count,r=right_count; if(dt_s<=0) return; speed.left=count_to_m(l-last_left)/dt_s; speed.right=count_to_m(r-last_right)/dt_s; last_left=l; last_right=r; }
EncoderCounts Encoder_GetCount(void) { EncoderCounts v={left_count,right_count}; return v; }
EncoderPair Encoder_GetSpeed(void) { return speed; }
EncoderPair Encoder_GetDistance(void) { EncoderPair v={count_to_m(left_count-zero_left),count_to_m(right_count-zero_right)}; return v; }
void Encoder_ResetDistance(void) { zero_left=left_count; zero_right=right_count; }
