# -*- coding: utf-8 -*-
"""
gimbal.py — 独立云台模式:MaixCAM 直接 PWM 驱动二自由度舵机 + PD 闭环
接线:
    MaixCAM/Pro:  yaw->A19(PWM7)  pitch->A18(PWM6)
    MaixCAM2:     yaw->A31(PWM7)  pitch->A30(PWM6)  [均在右排 2x6P]
                  右排每组自带 5V/GND, 舵机信号/电源/地可全从右排取。
                  注意: A30/A31 与 UART1 复用, 用作云台 PWM 时勿再开 UART1;
                        主控通信走左排 UART2(B0/B1), 与此不冲突, 可同时用。
    舵机由右排 5V 供电时务必与板共地; 大扭矩舵机建议外部独立 5V 电源+共地。
说明: 50Hz, 占空比 2.5%~12.5% 对应 0.5~2.5ms (0~180°)。
      本模块做增量式 PD:把目标点的像素误差收敛到画面中心。
      联调阶段把控制权交还主控时,不进 GIMBAL 模式即可,互不影响。
"""
from maix import pwm, pinmap, err, sys

SERVO_FREQ = 50


def _pins():
    if sys.device_id() == "maixcam2":
        # MaixCAM2 官方引脚图: 右排 2x6P 引出 PWM6=A30, PWM7=A31
        # 右排每组自带 5V/GND, 舵机电源可就近取(注意 5V 供电时该脚才是 5V)
        return ("A30", 6), ("A31", 7)   # (pitch, PWM6), (yaw, PWM7)
    return ("A18", 6), ("A19", 7)       # MaixCAM/Pro: (pitch), (yaw)


class Gimbal:
    def __init__(self, p):
        self.p = p
        g = p.d["gimbal"]
        (pin_pitch, id_pitch), (pin_yaw, id_yaw) = _pins()
        err.check_raise(pinmap.set_pin_function(pin_pitch, "PWM%d" % id_pitch), "pinmap pitch failed")
        err.check_raise(pinmap.set_pin_function(pin_yaw, "PWM%d" % id_yaw), "pinmap yaw failed")
        c = g["duty_center"]
        self.duty_yaw = c
        self.duty_pitch = c
        self.pwm_pitch = pwm.PWM(id_pitch, freq=SERVO_FREQ, duty=c, enable=True)
        self.pwm_yaw = pwm.PWM(id_yaw, freq=SERVO_FREQ, duty=c, enable=True)
        self.last_ex = 0.0
        self.last_ey = 0.0

    def _clamp(self, d):
        g = self.p.d["gimbal"]
        return max(g["duty_min"], min(g["duty_max"], d))

    def center(self):
        c = self.p.d["gimbal"]["duty_center"]
        self.duty_yaw = c
        self.duty_pitch = c
        self.pwm_yaw.duty(c)
        self.pwm_pitch.duty(c)
        self.last_ex = 0.0
        self.last_ey = 0.0

    def update(self, err_x, err_y):
        """err_x/err_y: 目标点相对画面中心的像素误差(右/下为正)。每帧调用一次。"""
        g = self.p.d["gimbal"]
        sx = -1.0 if g["inv_x"] else 1.0
        sy = -1.0 if g["inv_y"] else 1.0
        dx = g["kp"] * err_x + g["kd"] * (err_x - self.last_ex)
        dy = g["kp"] * err_y + g["kd"] * (err_y - self.last_ey)
        self.last_ex = err_x
        self.last_ey = err_y
        self.duty_yaw = self._clamp(self.duty_yaw + sx * dx)
        self.duty_pitch = self._clamp(self.duty_pitch + sy * dy)
        self.pwm_yaw.duty(self.duty_yaw)
        self.pwm_pitch.duty(self.duty_pitch)

    def hold(self):
        """无目标时保持当前位置并清微分,防止丢目标后猛甩。"""
        self.last_ex = 0.0
        self.last_ey = 0.0
