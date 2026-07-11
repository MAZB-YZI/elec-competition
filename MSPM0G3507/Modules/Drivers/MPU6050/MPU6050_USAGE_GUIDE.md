# MPU6050 在 H 题中的使用指南

## 1. MPU6050 的核心作用

MPU6050 不是"完整姿态传感器"，而是**小车的电子指南针**：
- 靠 Z 轴陀螺仪积分出短时间航向角
- 主要用于：直线抗偏、定角转向、转弯辅助

**分工原则：**
- **灰度传感器**：负责看线、找端点、半圆循迹
- **编码器**：负责距离、速度、左右轮闭环
- **MPU6050**：负责航向角、直线抗偏、定角转向

---

## 2. MPU6050 在 H 题中的应用场景

### 直线 A-B / C-D
保持车头方向不偏：
```c
target_yaw = 0.0f;
yaw_error = target_yaw - current_yaw;
turn = Kp_yaw * yaw_error + Kd_yaw * gyro_z;
left_pwm  = base_pwm - turn;
right_pwm = base_pwm + turn;
```

### 对角线 A-C / B-D
角度约 38.66°，距离约 1.2806m：
```c
target_yaw = +38.66f;  // 正负看车体坐标定义
```

### 定角转向
```c
Motion_TurnToAngle(38.66f);
// 内部逻辑：
error = angle_error(target_yaw, current_yaw);
if (abs(error) > threshold) {
    left_pwm  = -turn_pwm;
    right_pwm = +turn_pwm;
} else {
    stop;
}
```

### 半圆循迹
- 主要靠灰度循迹，MPU6050 只做辅助
- 防止入弯角度太离谱
- 判断大致转过了多少角度
- 防止灰度短暂丢线时车头乱飘

---

## 3. 推荐软件接口

```c
bool MPU6050_Init(void);
bool MPU6050_ReadRaw(MPU6050_RawData *raw);
bool MPU6050_CalibrateGyroZ(uint16_t samples);
bool MPU6050_UpdateYaw(float dt);
float MPU6050_GetYaw(void);
float MPU6050_GetGyroZ(void);
void MPU6050_ResetYaw(float yaw_deg);
```

数据结构：
```c
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} MPU6050_RawData;
```

---

## 4. Yaw 更新算法（含死区）

```c
bool MPU6050_UpdateYaw(float dt)
{
    MPU6050_RawData raw;
    if (!MPU6050_ReadRaw(&raw)) return false;

    gyro_z_dps = ((float)raw.gz - gyro_z_bias) / 131.0f;

    // 死区，减少静止漂移
    if (gyro_z_dps > -0.3f && gyro_z_dps < 0.3f) {
        gyro_z_dps = 0.0f;
    }

    yaw_deg += gyro_z_dps * dt;

    // 归一化到 -180 ~ +180
    if (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    else if (yaw_deg < -180.0f) yaw_deg += 360.0f;

    return true;
}
```

---

## 5. 航向误差计算（最短角度）

```c
float Angle_Error(float target, float current)
{
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}
```

---

## 6. 运动控制结合

直线控制：
```c
void Motion_StraightWithYaw(float target_yaw, float speed)
{
    float yaw = MPU6050_GetYaw();
    float error = Angle_Error(target_yaw, yaw);
    float turn = Kp_yaw * error - Kd_yaw * MPU6050_GetGyroZ();
    Motor_SetSpeed(speed - turn, speed + turn);
}
```

定角转向：
```c
bool Motion_TurnToYaw(float target_yaw)
{
    float yaw = MPU6050_GetYaw();
    float error = Angle_Error(target_yaw, yaw);
    if (fabsf(error) < 2.0f) { Motor_Stop(); return true; }
    float turn = Kp_turn * error;
    Motor_SetSpeed(-turn, turn);
    return false;
}
```

---

## 7. 调试显示建议

```text
WHO:68      MPU6050 是否通信成功
GZ:+012     当前 Z 轴角速度，deg/s
Y:+035      当前 yaw，deg
BZ:123      Z 轴零偏 raw
```

---

## 8. 最终比赛策略

1. 起点校准 MPU6050 零偏
2. 每 5ms 更新 yaw
3. 直线段：编码器控制距离，MPU6050 控制航向
4. 对角线：设定目标 yaw = ±38.66°
5. 半圆：灰度循迹为主，yaw 做辅助监控
6. 端点：灰度端点检测 + 编码器距离 + yaw 状态共同判断

**一句话：MPU6050 负责方向，编码器负责距离，灰度负责路线。**
