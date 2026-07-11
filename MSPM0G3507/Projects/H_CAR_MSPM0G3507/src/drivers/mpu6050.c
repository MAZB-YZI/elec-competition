#include "mpu6050.h"
#include "hcar_hal.h"
#define MPU_ADDR 0x68u
#define MPU_TIMEOUT 80000u
static float bias_z,yaw;
static bool wr(uint8_t r,uint8_t v){uint8_t b[2]={r,v};return HCarHal_I2CWrite(MPU_ADDR,b,2,MPU_TIMEOUT);}
static bool rd(uint8_t r,uint8_t*p,size_t n){return HCarHal_I2CWriteRead(MPU_ADDR,&r,1,p,n,MPU_TIMEOUT);}
bool MPU6050_Init(void){uint8_t id=0;yaw=bias_z=0;if(!rd(0x75,&id,1)||id!=0x68)return false;return wr(0x6B,1)&&wr(0x1A,3)&&wr(0x1B,0)&&wr(0x1C,0);}
bool MPU6050_ReadRaw(MPU6050Raw*r){uint8_t b[14];if(!r||!rd(0x3B,b,14))return false;int16_t*v=&r->accel_x;for(unsigned i=0;i<7;i++)v[i]=(int16_t)(((uint16_t)b[2*i]<<8)|b[2*i+1]);return true;}
bool MPU6050_CalibrateGyro(uint16_t n){int32_t sum=0;MPU6050Raw r;if(!n)return false;for(uint16_t i=0;i<n;i++){if(!MPU6050_ReadRaw(&r))return false;sum+=r.gyro_z;}bias_z=(float)sum/n;return true;}
bool MPU6050_Update(float dt){MPU6050Raw r;if(dt<=0||!MPU6050_ReadRaw(&r))return false;yaw+=((r.gyro_z-bias_z)/131.0f)*dt;return true;}
float MPU6050_GetYaw(void){return yaw;} void MPU6050_ResetYaw(void){yaw=0;}
