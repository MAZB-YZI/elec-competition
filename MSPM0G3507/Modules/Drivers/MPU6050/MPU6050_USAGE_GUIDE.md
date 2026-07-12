# MPU6050 在 H 题小车中的使用说明

## 1. 本模块的定位

MPU6050 在 H 题里不要当“完整姿态传感器”用，先把它当作短时间航向保持传感器：

- 用 Z 轴陀螺仪积分得到 yaw（航向角）。
- 直线和对角线段用 yaw 做抗偏。
- 定角转向用 yaw 判断是否转到目标角。
- 半圆循迹仍然以灰度传感器为主，MPU6050 只做辅助监督。

距离和速度由编码器负责，路线和端点由灰度传感器负责，MPU6050 只负责“车头方向”。

## 2. 当前硬件连接

天猛星拓展板上：

- OLED：硬件 I2C0，PA28=SDA，PA31=SCL。
- MPU6050：PA0=SDA，PA1=SCL。

PA0/PA1 和 PA28/PA31 都是 I2C0 的不同复用引脚组，不能同时作为两个硬件 I2C0 使用。为了避免飞线，本模块保留 OLED 使用硬件 I2C0，MPU6050 使用 `soft_i2c.c` 通过 GPIO 模拟 I2C。

软件 I2C 必须按 I2C 开漏思路处理：低电平主动拉低，高电平释放总线，依赖 3.3 V 上拉。不要把 SDA/SCL 当普通推挽 GPIO 硬推高。

## 3. 为什么暂时不直接搬 DMP

参考资料里的 DMP 工程适合读取 pitch/roll/yaw，但当前 H 题调试阶段更需要稳定、可控、可恢复：

- DMP 初始化代码量大，依赖完整 InvenSense 移植层。
- 参考工程里有阻塞式 `while(DMP_Init())`、`while(DMP_Read_Data(...))`，不适合当前容易遇到调试口连接问题的板级调试。
- H 题短时间航向控制主要需要 Z 轴陀螺仪积分，不一定需要完整 DMP 姿态解算。

后续如果 raw gyro yaw 已验证稳定，再把 DMP 作为增强模块单独移植，不要混进基础 bring-up 版本。

## 4. 推荐初始化和测试流程

1. 上电后先不要启动电机。
2. 调用 `SoftI2C_Init()` 释放并恢复 PA0/PA1 总线。
3. 调用 `MPU6050_Init()`，检查 `MPU6050_GetWhoAmI()` 是否为 `0x68`。
4. 保持小车静止，调用 `MPU6050_CalibrateGyro(200)` 或更多样本。
5. 以固定周期调用 `MPU6050_Update(dt_s)`，例如 5 ms 或 10 ms。
6. OLED/串口显示 `WHO`、`GZ`、`Yaw`、`Bias`、`Err`。

如果 `WHO` 读不到或 `Err` 持续增加，优先检查：

- MPU6050 供电和 GND。
- PA0/PA1 是否接反。
- SDA/SCL 上拉是否到 3.3 V。
- PA0/PA1 是否被其它外设或测试程序占用。

## 5. 当前接口

```c
bool MPU6050_Init(void);
bool MPU6050_ReadRaw(MPU6050Raw *raw);
bool MPU6050_CalibrateGyro(uint16_t samples);
bool MPU6050_Update(float dt_s);

float MPU6050_GetYaw(void);          // -180 ~ +180 deg
float MPU6050_GetGyroZ(void);        // deg/s
float MPU6050_GetGyroZBias(void);    // raw LSB
uint8_t MPU6050_GetWhoAmI(void);     // 正常为 0x68
uint32_t MPU6050_GetReadErrorCount(void);

void MPU6050_ResetYaw(void);
void MPU6050_ResetYawTo(float yaw_deg);
```

## 6. Yaw 更新算法

当前模块使用 ±250 dps 量程：

```c
gyro_z_dps = ((float)raw.gyro_z - gyro_z_bias_raw) / 131.0f;

if (gyro_z_dps > -0.3f && gyro_z_dps < 0.3f) {
    gyro_z_dps = 0.0f;
}

yaw_deg += gyro_z_dps * dt_s;
```

更新后会把 `yaw_deg` 归一化到 `-180 ~ +180`。

注意：yaw 是积分量，会随时间漂移。比赛策略上应在确定的几何节点、直线起点、端点或路线状态切换时重置或校正 yaw，不要指望 MPU6050 单独长期保持绝对方向。

## 7. 航向误差计算

运动控制层不要直接用 `target - current` 后不处理跨 ±180° 的情况。推荐：

```c
float Angle_Error(float target, float current)
{
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}
```

## 8. H 题控制用法

直线或对角线：

```c
float yaw = MPU6050_GetYaw();
float error = Angle_Error(target_yaw, yaw);
float turn = kp_yaw * error - kd_yaw * MPU6050_GetGyroZ();

left_speed  = base_speed - turn;
right_speed = base_speed + turn;
```

定角转向：

```c
float error = Angle_Error(target_yaw, MPU6050_GetYaw());
if (fabsf(error) < 2.0f) {
    Motor_Stop();
    done = true;
} else {
    float turn = kp_turn * error;
    Motor_SetSpeed(-turn, turn);
}
```

建议先在轮子离地或电机断电的情况下只测试 `WHO/GZ/Yaw`，确认手动左右转动小车时 GZ 和 Yaw 有变化，再接入运动闭环。
