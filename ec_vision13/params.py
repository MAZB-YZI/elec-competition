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
               "duty_min": 3.0, "duty_max": 12.0, "duty_center": 7.5},
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
    5: [("Kp", ("gimbal", "kp"), 0.0005, 0.0, 0.05),
        ("Kd", ("gimbal", "kd"), 0.0005, 0.0, 0.05),
        ("InvX", ("gimbal", "inv_x"), 1, 0, 1),
        ("InvY", ("gimbal", "inv_y"), 1, 0, 1)],
}
