# -*- coding: utf-8 -*-
"""
params.py — 参数中心:默认值 + JSON 持久化 + 触屏可编辑参数注册表
参数文件: /root/ec_vision_params.json (掉电保留, 开机自动加载)
"""
import json
import os
import copy

PARAM_FILE = "/root/ec_vision_params.json"

DEFAULTS = {
    "global": {
        "cam_w": 320, "cam_h": 240, "cam_fps": 60,
        "exp_us": 0,            # 0=自动曝光(默认, 和官方例程一致: 不重影不发绿)
                                # >0 才手动曝光(仅激光点等需压暗背景的特殊场景才开, 会触发手动白平衡)
        "gain": 300,            # 手动增益, 仅手动曝光(exp_us>0)时生效
        "awb_manual": 0,        # 1=锁白平衡
        "wb_gain": [0.0682, 0, 0, 0.04897],  # MaixCAM2 官方默认 R,Gr,Gb,B (手动曝光时自动套用)
        "uart_baud": 115200,
        "wdt_ms": 0,            # 0=关看门狗(开发期); 比赛设 5000
        "send_div": 1,          # 结果帧发送分频(1=每帧都发)
    },
    # LAB 阈值格式: [L_min, L_max, A_min, A_max, B_min, B_max]
    "line":   {"thr": [0, 45, -128, 127, -128, 127], "area_th": 100, "show_hits": 1},
    "blob":   {"thr": [0, 80, 40, 80, 10, 80], "area_th": 200, "pixels_th": 200},
    "target": {"method": "centroid",    # 兼容旧字段; 实际由 method_idx 决定
               "method_idx": 0,          # 0=centroid质心(默认) 1=blob最大块 2=circle圆检测
               "pick_which_idx": 0,      # 取色目标: 0=靶心 1=激光(PickWhat 切换)
               "center_thr": [0, 100, 30, 127, 20, 127],   # 靶心主色(默认偏红), 触屏可调
               "center_area": 300,
               "circle_th": 3000, "r_min": 5, "r_max": 60,  # method=circle 时用
               "laser_thr": [60, 100, 20, 80, -10, 60],
               "target_w_cm": 26.0},   # 靶纸实际宽度/直径, 用于像素->厘米粗换算
    "detect": {"model": "/root/models/test.mud", "conf": 0.5, "preload": 0},
    "circle": {"threshold": 3000, "r_min": 10, "r_max": 100, "downscale": 2},
    "gimbal": {"kp": 0.0040, "kd": 0.0010, "inv_x": 0, "inv_y": 0,
               "duty_min": 3.0, "duty_max": 12.0, "duty_center": 7.5,
               "drv": 1,           # 云台驱动: 1=F32C无刷总线电机(实际硬件) 0=PWM舵机(备用)
                                   # 改 drv 后重启程序生效(驱动只在进 GIMB 时初始化一次)
               # 目标点/瞄准点组合(详见 main.py _proc_gimbal 注释):
               #   0=色块/中心+AimOff  1=靶心/中心+AimOff  2=靶心/实测激光★  3=6cm圆动点/实测激光★
               "src": 0,
               "circ_r": 40,       # Src=3 画圆半径(px)。题目要 6cm, 先按固定距离手标;
                                   #   靶心检测给的 size 可换算 px/cm, 后续可做自动标定
               "circ_t": 20.0,     # Src=3 转一圈的秒数(题目: 小车1圈20s)
               "dead_px": 3,       # 死区(px): 误差小于此值云台不动, 消抖振
               "max_step": 0.25,   # [仅PWM] 单帧占空比最大步进(%), 防猛甩
               "lock_px": 6,       # |ex|,|ey| 同时小于此值连续10帧 => 屏显 LOCK
               "aim_off_x": 0,     # 瞄准点偏移(px): 激光器与相机光轴不重合时的校靶量
               "aim_off_y": 0,
               "dbg": 0},          # 1=GIMB 每帧打印 GIMBLOG,帧号,ex,ey,yaw,pitch (整定用)
    # WHEELTEC F32C TTL 无刷云台电机(drv=1 时生效), 协议见 f32c.py 文件头
    "f32c":  {"yaw_id": 1, "pitch_id": 2,      # 电机总线地址(厂商软件预设 YAW=1 PITCH=2)
              "baud": 115200,
              "axes": 2,                       # 在线轴: 3=双轴 1=仅YAW(ID1) 2=仅PITCH(ID2)
                                               # 当前=2: ID1(X/YAW)已被60V烧毁, 新电机到货后改回3
                                               # 电机烧坏/拆掉时必须设对, 否则驱动会去戳死地址

              "pos_mode": 3,                   # 位置模式: 3=多圈直通(手册推荐高频改目标) 1=多圈T型(官方例程)
                                               # 改后重启生效; 直通感觉发抽再切回1
              "speed_rpm": 30,                 # 位置模式限速(参考工程默认10偏慢, 30更跟手; 上限1000)
              # ★ 视觉外环增益。注意 update() 里 ang 是累加的, 所以:
              #    kp 乘的是【误差累加和】= 积分增益 I (是它决定稳定性, 太大必振荡)
              #    kd 乘的是【当前误差】  = 比例增益 P
              #    这是 PI 环不是 PD 环。振荡先降 kp, 不是加 kd。
              # 0.02/0.0 对 1~4 帧的任意环路延迟都稳(|z|<=0.90), 先跑通再往上加。
              "kp": 0.02, "kd": 0.0,
              "max_step_deg": 3.0,             # 单帧角度步进上限(度)
              "yaw_lim": 60.0, "pitch_lim": 40.0},  # 软件限位(度): 保护相机排线, 必设!
}


def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        elif k in dst:
            dst[k] = v


class Params:
    def __init__(self):
        self.d = copy.deepcopy(DEFAULTS)
        self.load()

    def load(self):
        try:
            if os.path.exists(PARAM_FILE):
                with open(PARAM_FILE, "r") as f:
                    _merge(self.d, json.load(f))
                print("[params] loaded", PARAM_FILE)
        except Exception as e:
            print("[params] load failed:", e)

    def save(self):
        try:
            with open(PARAM_FILE, "w") as f:
                json.dump(self.d, f, indent=1)
            os.sync()
            print("[params] saved")
            return True
        except Exception as e:
            print("[params] save failed:", e)
            return False

    # path 形如 ("blob","thr",2) 或 ("gimbal","kp")
    def get(self, path):
        v = self.d
        for p in path:
            v = v[p]
        return v

    def set(self, path, val):
        v = self.d
        for p in path[:-1]:
            v = v[p]
        v[path[-1]] = val


# ---- 触屏可编辑参数注册表: 每模式一组 (显示名, path, 步进, 最小, 最大) ----
LAB_NAMES = ["Lmin", "Lmax", "Amin", "Amax", "Bmin", "Bmax"]


def _lab_entries(section, key, step=5):
    lim = [(-0, 100), (0, 100), (-128, 127), (-128, 127), (-128, 127), (-128, 127)]
    return [(LAB_NAMES[i], (section, key, i), step, lim[i][0], lim[i][1]) for i in range(6)]


EDITABLE = {
    0: [  # IDLE -> 全局相机参数
        ("ExpUs", ("global", "exp_us"), 100, 0, 40000),
        ("Gain", ("global", "gain"), 50, 0, 4000),
        ("WB_R", ("global", "wb_gain", 0), 0.01, 0.0, 1.0),
        ("WB_B", ("global", "wb_gain", 3), 0.01, 0.0, 1.0),
        ("AWBloc", ("global", "awb_manual"), 1, 0, 1),
        ("Fps", ("global", "cam_fps"), 30, 30, 90),
        ("Baud", ("global", "uart_baud"), 115200, 115200, 921600),
    ],
    1: _lab_entries("line", "thr") + [("AreaTh", ("line", "area_th"), 50, 0, 20000)],
    2: _lab_entries("blob", "thr") + [
        ("AreaTh", ("blob", "area_th"), 50, 0, 20000),
        ("PixTh", ("blob", "pixels_th"), 50, 0, 20000),
    ],
    3: [("TgtWcm", ("target", "target_w_cm"), 0.5, 1.0, 200.0),
        ("Method", ("target", "method_idx"), 1, 0, 2),
        ("PickWhat", ("target", "pick_which_idx"), 1, 0, 1),
        ("CtrArea", ("target", "center_area"), 50, 0, 20000)]
       + [("C" + LAB_NAMES[i], ("target", "center_thr", i), 5,
          (-0 if i < 2 else -128), (100 if i < 2 else 127)) for i in range(6)]
       + [("L" + LAB_NAMES[i], ("target", "laser_thr", i), 5,
          (-0 if i < 2 else -128), (100 if i < 2 else 127)) for i in range(6)],
    4: [("Conf", ("detect", "conf"), 0.05, 0.05, 0.95)],
    6: [("CircTh", ("circle", "threshold"), 200, 500, 20000),
        ("Rmin", ("circle", "r_min"), 2, 1, 200),
        ("Rmax", ("circle", "r_max"), 5, 5, 300),
        ("DownS", ("circle", "downscale"), 1, 1, 4)],
    5: [("Src", ("gimbal", "src"), 1, 0, 3),
        ("CircR", ("gimbal", "circ_r"), 2, 5, 150),
        ("CircT", ("gimbal", "circ_t"), 1.0, 2.0, 60.0),
        ("Drv", ("gimbal", "drv"), 1, 0, 1),
        ("PosMd", ("f32c", "pos_mode"), 2, 1, 3),
        ("Axes", ("f32c", "axes"), 1, 1, 3),
        ("Dbg", ("gimbal", "dbg"), 1, 0, 1),
        ("FKp", ("f32c", "kp"), 0.01, 0.0, 0.5),
        ("FKd", ("f32c", "kd"), 0.005, 0.0, 0.2),
        ("FSpd", ("f32c", "speed_rpm"), 5, 5, 100),
        ("FStep", ("f32c", "max_step_deg"), 0.5, 0.5, 10.0),
        ("YawLim", ("f32c", "yaw_lim"), 5, 10, 170),
        ("PitLim", ("f32c", "pitch_lim"), 5, 5, 80),
        ("InvX", ("gimbal", "inv_x"), 1, 0, 1),
        ("InvY", ("gimbal", "inv_y"), 1, 0, 1),
        ("DeadPx", ("gimbal", "dead_px"), 1, 0, 30),
        ("LockPx", ("gimbal", "lock_px"), 1, 1, 50),
        ("AimOfX", ("gimbal", "aim_off_x"), 2, -160, 160),
        ("AimOfY", ("gimbal", "aim_off_y"), 2, -120, 120),
        ("Kp", ("gimbal", "kp"), 0.0005, 0.0, 0.05),
        ("Kd", ("gimbal", "kd"), 0.0005, 0.0, 0.05),
        ("MaxStep", ("gimbal", "max_step"), 0.05, 0.05, 1.0)],
}
