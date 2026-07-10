#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

/* ---- encoder & wheel params ---- */
#define EncoderMultiples  2              // encoder multiplier, 2x on TI platform
#define CONTROL_FREQUENCY 200            // encoder read frequency
#define Black_WheelDiameter   0.065f     // wheel diameter (m)
#define Perimeter       0.204203519f     // wheel perimeter (m)
#define MOTOR_GEAR_RATIO       28.0f     // motor reduction ratio
#define ENCODER_RESOLUTION     13.0f     // encoder lines
#define Wheelspacing    0.1610f          // wheel track width (m)
#define PI              3.1415926f

/* ---- motor PWM limit ---- */
#define PWM_MAX         7800

/* ---- work mode select ---- */
#define CONTROL_MODE_RUN     0      /* 巡线打靶模式 */
#define CONTROL_MODE_SEARCH  1      /* 寻靶模式 */

/* ---- encoder sign (negative = flip direction) ---- */
#define LEFT_ENCODER_SIGN    1.0f
#define RIGHT_ENCODER_SIGN   1.0f
#define LEFT_PWM_SIGN       -1
#define RIGHT_PWM_SIGN       1

/* ================================================================
 *  Vision / Gimbal PD defines
 * ================================================================ */
#define VISION_DIR_X              -1.0f  /* X error -> yaw output sign */
#define VISION_DIR_Y              -1.0f  /* Y error -> pitch output sign */
#define VISION_DEADZONE            3.0f  /* deadzone near target center (pixels) */
#define VISION_LOST_TICKS          40    /* consecutive lost-frame limit */
#define VISION_YAW_LIMIT_RPM       30.0f /* vision yaw PD output limit */
#define VISION_PITCH_LIMIT_RPM     10.0f /* vision pitch PD output limit */
#define VISION_PITCH_STEP_LIMIT_X10 10.0f /* pitch angle per-cycle increment limit */
#define GIMBAL_YAW_COMP_LIMIT_RPM  35.0f /* IMU z-axis yaw compensation limit */
#define GIMBAL_YAW_LIMIT_RPM       100.0f /* final yaw speed limit sent to gimbal */
#define VISION_FPS_SAMPLE_TICKS    200   /* frame-rate sample window */

/* ---- motor param struct ---- */
typedef struct
{
    float Current_Encoder;      /* encoder value, real-time speed */
    float Motor_Pwm;            /* motor PWM value */
    float Target_Encoder;       /* target encoder speed */
    float Velocity;             /* motor velocity */
} Motor_parameter;

/* ---- encoder struct ---- */
typedef struct
{
    int A;
    int B;
} Encoder;

/* ---- motor / encoder / control globals ---- */
extern float Move_X, Move_Z;
extern Encoder OriginalEncoder;
extern volatile Motor_parameter MotorA, MotorB;
extern float Voltage_Count, Voltage_All, Voltage;
extern float Velocity_KP, Velocity_KI;
extern int Run_Mode;
extern u8 Control_Work_Mode;
extern u8 Control_Work_Enable;

/* ---- vision PD params ---- */
extern float KP_VISION_X;
extern float KD_VISION_X;
extern float KP_VISION_Y;
extern float KD_VISION_Y;
extern float AC_VISION_K;
extern float CD_VISION_K;
extern float DB_VISION_K;
extern float CD_VISION_FF;

/* ---- vision / gimbal run state ---- */
extern int VISION_CENTER_X;
extern int VISION_CENTER_Y;
extern volatile u16 Vision_Fps;
extern volatile u8  Vision_Online;
extern volatile int16_t Gimbal_Joint1_Target_Rpm;
extern volatile int16_t Gimbal_Joint2_Target_Rpm;
extern volatile uint16_t Gimbal_Joint2_Target_Angle;

/* ---- function declarations ---- */
void Get_Velocity_From_Encoder(int Encoder1, int Encoder2);
float target_limit_float(float insert, float low, float high);
int target_limit_int(int insert, int low, int high);
void Get_Target_Encoder(float Vx, float Vz);
int Incremental_PI_Left(float Encoder, float Target);
int Incremental_PI_Right(float Encoder, float Target);
void Set_Pwm(int motor_a, int motor_b);
int Turn_Off(void);
int myabs(int a);
void Key(void);

#endif
