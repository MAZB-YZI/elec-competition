# -*- coding: utf-8 -*-
"""
main.py — 电赛视觉框架 (MaixCAM / MaixPy v4)
模式: 0 IDLE | 1 LINE 循迹 | 2 BLOB 色块/激光点 | 3 TARGET 靶纸 | 4 DETECT AI识别 | 5 GIMBAL 独立云台
切换: 触屏顶栏按钮, 或主控串口 0x80 SET_MODE 指令 (见 PROTOCOL.md)
运行: MaixVision 连板后直接运行本文件; 比赛封版时打包安装为 app 并设开机自启 (见 README.md)
"""
import threading
import struct

from maix import camera, display, image, touchscreen, app, time, uart, pinmap, err, sys, wdt

import proto
from params import Params, EDITABLE
from gimbal import Gimbal

MODE_IDLE, MODE_LINE, MODE_BLOB, MODE_TARGET, MODE_DETECT, MODE_GIMBAL, MODE_CIRCLE, MODE_TAG = range(8)
MODE_NAMES = ["IDLE", "LINE", "BLOB", "TARG", "DET", "GIMB", "CIRC", "TG/QR"]
BAUDS = [115200, 230400, 460800, 921600]


def log(*a):
    print("[%d]" % time.ticks_ms(), *a)


class Fps:
    def __init__(self):
        self.t = time.ticks_ms()
        self.n = 0
        self.val = 0

    def tick(self):
        self.n += 1
        now = time.ticks_ms()
        if now - self.t >= 1000:
            self.val = self.n * 1000 // max(1, now - self.t)
            self.n = 0
            self.t = now
        return self.val


class Button:
    def __init__(self, x, y, w, h, label, cb):
        self.pos = [x, y, w, h]
        self.label = label
        self.cb = cb
        self.disp_pos = None

    def map_disp(self, iw, ih, dw, dh):
        self.disp_pos = image.resize_map_pos(iw, ih, dw, dh, image.Fit.FIT_CONTAIN,
                                             self.pos[0], self.pos[1], self.pos[2], self.pos[3])

    def hit(self, x, y):
        p = self.disp_pos
        return p and (p[0] < x < p[0] + p[2]) and (p[1] < y < p[1] + p[3])

    def draw(self, img, active=False):
        c = image.COLOR_GREEN if active else image.COLOR_WHITE
        img.draw_rect(self.pos[0], self.pos[1], self.pos[2], self.pos[3], c, 1)
        img.draw_string(self.pos[0] + 4, self.pos[1] + 4, self.label, c)


class VisionApp:
    UI_TOP = 24     # 顶栏模式按钮+状态行高度, ROI 上边界排除
    UI_BOT = 24     # 底栏调参按钮高度, ROI 下边界排除

    def __init__(self):
        self.p = Params()
        g = self.p.d["global"]

        self.disp = display.Display()
        self.ts = touchscreen.TouchScreen()
        self.cam = None
        self._open_camera()

        # ---- UART ----
        if sys.device_id() == "maixcam2":
            # MaixCAM2 排针丝印 U2T/U2R = UART2, 引脚 B0/B1, 节点 /dev/ttyS2
            pins = {"B0": "UART2_TX", "B1": "UART2_RX"}
            dev = "/dev/ttyS2"
        else:
            pins = {"A16": "UART0_TX", "A17": "UART0_RX"}
            dev = "/dev/ttyS0"
        for pin, func in pins.items():
            err.check_raise(pinmap.set_pin_function(pin, func), "pinmap %s failed" % pin)
        self.uart = uart.UART(dev, int(g["uart_baud"]))
        self.parser = proto.Parser()
        self.rx_lock = threading.Lock()
        self.rx_cmds = []
        self.uart.set_received_callback(self._on_uart)

        # ---- 状态 ----
        self.mode = MODE_IDLE
        self.fps = Fps()
        self.frame_i = 0
        self.last_hb = 0
        self.cam_fail = 0
        self.param_idx = 0
        self._param_dirty = False   # 有未存盘的参数改动(不按 SV 则重启丢失)
        self.detector = None
        self.det_err = ""
        self.gimbal = None
        self.gim_err = ""
        self.lock_n = 0          # GIMB: 连续对准帧计数, 达阈值屏显 LOCK
        self.last_qr = ""

        # ---- 看门狗 (wdt_ms=0 关闭) ----
        self.wd = None
        if int(g["wdt_ms"]) > 0:
            self.wd = wdt.WDT(0, int(g["wdt_ms"]))
            log("WDT on:", g["wdt_ms"], "ms")

        if int(self.p.d["detect"]["preload"]):
            self._ensure_detector()

        self._build_ui()
        log("init done, dev=%s baud=%d" % (dev, int(g["uart_baud"])))

    # ---------------- 硬件 ----------------
    def _open_camera(self):
        g = self.p.d["global"]
        for _ in range(50):
            try:
                self.cam = camera.Camera(int(g["cam_w"]), int(g["cam_h"]), fps=int(g["cam_fps"]))
                self._apply_cam_params()
                return
            except Exception as e:
                log("camera open failed:", e)
                time.sleep_ms(300)
        raise RuntimeError("camera open failed too many times")

    def _apply_cam_params(self):
        g = self.p.d["global"]
        try:
            manual_exp = int(g["exp_us"]) > 0
            if manual_exp:
                self.cam.exposure(int(g["exp_us"]))       # 设定后自动切手动曝光
                if int(g.get("gain", 0)) > 0:
                    self.cam.gain(int(g["gain"]))         # 手动增益仅手动曝光下生效, 补偿短曝光的暗
            else:
                self.cam.exp_mode(camera.AeMode.Auto)
            # 关键: 手动曝光会破坏自动白平衡(画面发绿), 故手动曝光时必须手动指定白平衡
            if int(g["awb_manual"]) or manual_exp:
                self.cam.awb_mode(camera.AwbMode.Manual)
                self.cam.set_wb_gain(list(g["wb_gain"]))
            else:
                self.cam.awb_mode(camera.AwbMode.Auto)
        except Exception as e:
            log("apply cam params failed:", e)

    def _detect_model_type(self, mud_path):
        """读取 .mud 里的 model_type, 返回对应的 nn 类名。MaixHub 训练可能出 yolov5/v8/11。"""
        try:
            with open(mud_path, "r") as f:
                txt = f.read().lower()
            for key, cls in (("yolov5", "YOLOv5"), ("yolov8", "YOLOv8"),
                             ("yolo11", "YOLO11"), ("yolov11", "YOLO11"),
                             ("yolo26", "YOLO26")):
                if key in txt:
                    return cls
        except Exception as e:
            log("read mud type err:", repr(e))
        return "YOLO11"     # 默认

    def _ensure_detector(self):
        if self.detector is not None:
            return True
        try:
            if self.wd:
                self.wd.feed()
            from maix import nn
            mud = self.p.d["detect"]["model"]
            cls_name = self._detect_model_type(mud)     # 按 mud 里的 model_type 选类
            YoloCls = getattr(nn, cls_name)
            self.detector = YoloCls(model=mud, dual_buff=True)
            if self.wd:
                self.wd.feed()
            self.det_err = ""
            log("model loaded:", mud, "as", cls_name)
            return True
        except Exception as e:
            self.det_err = str(e)[:40]
            log("model load failed:", e)
            return False

    def _ensure_gimbal(self):
        if self.gimbal is not None:
            return True
        try:
            if int(self.p.d["gimbal"].get("drv", 1)) == 1:
                from f32c import F32CGimbal      # F32C 无刷总线电机(UART1, A30/A31)
                self.gimbal = F32CGimbal(self.p)
            else:
                self.gimbal = Gimbal(self.p)     # PWM 舵机备用方案(PWM6/7, A30/A31)
            self.gim_err = ""
            return True
        except Exception as e:
            self.gim_err = str(e)[:40]
            log("gimbal init failed:", e)
            return False

    # ---------------- UART ----------------
    def _on_uart(self, serial, data: bytes):
        try:
            frames = self.parser.feed(data)
            if frames:
                with self.rx_lock:
                    self.rx_cmds.extend(frames)
        except Exception as e:
            log("uart cb err:", e)

    def _handle_rx(self):
        with self.rx_lock:
            cmds, self.rx_cmds = self.rx_cmds, []
        for cmd, payload in cmds:
            if cmd == proto.CMD_SET_MODE and len(payload) >= 1 and payload[0] <= MODE_TAG:
                self._switch_mode(payload[0])
                self.uart.write(proto.pack_heartbeat(self.mode, self.fps.val))
            elif cmd == proto.CMD_PING:
                self.uart.write(proto.pack_heartbeat(self.mode, self.fps.val))

    def _send(self, pkt):
        if pkt and self.frame_i % max(1, int(self.p.d["global"]["send_div"])) == 0:
            try:
                self.uart.write(pkt)
            except Exception as e:
                log("uart tx err:", e)

    # ---------------- 模式 ----------------
    def _switch_mode(self, m):
        if m == self.mode:
            return
        log("mode %s -> %s" % (MODE_NAMES[self.mode], MODE_NAMES[m]))
        self.mode = m
        self.param_idx = 0
        if m == MODE_DETECT:
            self._ensure_detector()
        if m == MODE_GIMBAL:
            self.lock_n = 0
            self.gimb_n = 0
            if self._ensure_gimbal():
                self.gimbal.center()
            # 打印【实际生效】的参数: /root/ec_vision_params.json 里存的值会覆盖代码里的默认值,
            # 所以"我改了默认值"不等于"你跑的就是这个值"。每次进 GIMB 都亮出来, 不用猜。
            g = self.p.d["gimbal"]
            f = self.p.d["f32c"]
            log("GIMB params: Drv=%d Axes=%d Src=%d FKp=%.3f FKd=%.3f FSpd=%d FStep=%.1f Dbg=%d" % (
                int(g.get("drv", 1)), int(f.get("axes", 3)), int(g.get("src", 0)),
                f.get("kp", 0), f.get("kd", 0), int(f.get("speed_rpm", 30)),
                f.get("max_step_deg", 3.0), int(g.get("dbg", 0))))
            if not int(g.get("dbg", 0)):
                log("GIMB: Dbg=0 -> 不打印 GIMBLOG。要整定请在 GIMB 页把 Dbg 调成 1(第5个参数)")

    @staticmethod
    def _bbox_from_corners(corners):
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
        return x0, y0, x1 - x0, y1 - y0

    def _find_biggest(self, img, thr, area_th, pixels_th, color):
        try:
            blobs = img.find_blobs([list(thr)], area_threshold=int(area_th),
                                   pixels_threshold=int(pixels_th), roi=self._roi(img))
        except Exception:
            # 某些固件 roi 关键字不兼容, 退回无 roi(丢失 UI 过滤但不崩)
            blobs = img.find_blobs([list(thr)], area_threshold=int(area_th),
                                   pixels_threshold=int(pixels_th))
        best, best_area = None, -1
        for b in blobs:
            x, y, w, h = self._bbox_from_corners(b.corners())
            if w * h > best_area:
                best_area = w * h
                best = (x, y, w, h)
        if best:
            x, y, w, h = best
            img.draw_rect(x, y, w, h, color, 2)
        return best

    def _roi(self, img):
        """排除顶栏/底栏 UI 区域, 只在画面中间找目标, 避免 UI 像素污染算法。"""
        return [0, self.UI_TOP, img.width(), img.height() - self.UI_TOP - self.UI_BOT]

    def _draw_center_cross(self, img, err_x=None, err_y=None, aim=None):
        """瞄准点十字 + 偏差条。aim=None 时瞄准点=画面中心(LINE 模式沿用不受影响);
        GIMB 模式传 aim=(画面中心+AimOff), 双轴偏差条一眼看清收敛方向和大小。"""
        ax = aim[0] if aim else img.width() // 2
        ay = aim[1] if aim else img.height() // 2
        img.draw_line(ax, self.UI_TOP, ax, img.height() - self.UI_BOT, image.COLOR_GRAY, 1)
        img.draw_line(ax - 8, ay, ax + 8, ay, image.COLOR_YELLOW, 1)
        img.draw_line(ax, ay - 8, ax, ay + 8, image.COLOR_YELLOW, 1)
        if err_x is not None:
            bx = max(-ax, min(img.width() - ax, int(err_x)))    # 横向偏差条
            col = image.COLOR_GREEN if abs(err_x) < 15 else image.COLOR_RED
            img.draw_line(ax, ay + 14, ax + bx, ay + 14, col, 3)
        if err_y is not None:
            by = max(-ay, min(img.height() - ay, int(err_y)))   # 纵向偏差条
            col = image.COLOR_GREEN if abs(err_y) < 15 else image.COLOR_RED
            img.draw_line(ax + 14, ay, ax + 14, ay + by, col, 3)

    def _proc_line(self, img):
        c = self.p.d["line"]
        roi = self._roi(img)
        # 调试: 把命中阈值的所有区域描出来(红框), 直观看到阈值到底框住了什么
        if c.get("show_hits", 1):
            try:
                hits = img.find_blobs([list(c["thr"])], area_threshold=20, pixels_threshold=20, roi=roi)
                for b in hits[:20]:
                    r = b.rect() if hasattr(b, "rect") else None
                    if r:
                        img.draw_rect(r[0], r[1], r[2], r[3], image.COLOR_RED, 1)
            except Exception:
                pass
        try:
            lines = img.get_regression([list(c["thr"])], area_threshold=int(c["area_th"]), roi=roi)
        except Exception:
            lines = img.get_regression([list(c["thr"])], area_threshold=int(c["area_th"]))
        if not lines:
            self._draw_center_cross(img)
            return proto.pack_line(0, 0, False), "no line"
        a = lines[0]
        img.draw_line(a.x1(), a.y1(), a.x2(), a.y2(), image.COLOR_GREEN, 2)
        theta = a.theta()
        theta = 270 - theta if theta > 90 else 90 - theta   # 官方例程同款角度映射
        err_x = (a.x1() + a.x2()) // 2 - img.width() // 2   # 线中点相对画面中心的横向偏差
        self._draw_center_cross(img, err_x)
        return proto.pack_line(err_x, theta, True), "err=%d ang=%d" % (err_x, theta)

    def _proc_blob(self, img):
        c = self.p.d["blob"]
        best = self._find_biggest(img, c["thr"], c["area_th"], c["pixels_th"], image.COLOR_RED)
        if not best:
            return proto.pack_blob(0, 0, 0, 0, False), "no blob", None
        x, y, w, h = best
        cx, cy = x + w // 2, y + h // 2
        img.draw_line(cx - 6, cy, cx + 6, cy, image.COLOR_RED, 1)
        img.draw_line(cx, cy - 6, cx, cy + 6, image.COLOR_RED, 1)
        return proto.pack_blob(cx, cy, w, h, True), "c=(%d,%d)" % (cx, cy), (cx, cy)

    def _find_target_center(self, img):
        """找靶心。mode=blob: 靶纸主色最大色块中心(最稳); mode=circle: find_circles 找圆环。
        返回 (cx, cy, size) 或 None。不使用 find_rects(MaixCAM2 固件下会段错误)。"""
        c = self.p.d["target"]
        method = ["centroid", "blob", "circle"][int(c.get("method_idx", 0)) % 3]
        y_lo, y_hi = self.UI_TOP, img.height() - self.UI_BOT
        if method == "circle":
            cc = self.p.d["circle"]      # 复用独立 CIRC 模式的圆检测参数
            ds = max(1, min(4, int(cc.get("downscale", 2))))
            try:
                if ds > 1:
                    small = img.resize(img.width() // ds, img.height() // ds)
                    circles = small.find_circles(threshold=int(cc["threshold"]),
                                                 r_min=max(1, int(cc["r_min"]) // ds),
                                                 r_max=max(2, int(cc["r_max"]) // ds))
                else:
                    circles = img.find_circles(threshold=int(cc["threshold"]),
                                               r_min=int(cc["r_min"]), r_max=int(cc["r_max"]))
            except Exception as e:
                log("find_circles err:", repr(e))
                return None
            best, best_r = None, -1
            for ci in circles:
                cx, cy, r = ci.x() * ds, ci.y() * ds, ci.r() * ds
                if cy < y_lo or cy > y_hi:
                    continue
                if r > best_r:
                    best_r, best = r, (cx, cy, r * 2)
            if best:
                img.draw_circle(best[0], best[1], best[2] // 2, image.COLOR_GREEN, 2)
            return best
        if method == "blob":
            # 找最大单色块中心(适合实心色块靶, 不适合同心圆环)
            blob = self._find_biggest(img, c["center_thr"], int(c["center_area"]),
                                      int(c["center_area"]), image.COLOR_GREEN)
            if not blob:
                return None
            x, y, w, h = blob
            return (x + w // 2, y + h // 2, w)
        # 默认 centroid: 所有靶色块的加权质心。同心圆环的各段红弧质心汇聚于圆心, 稳定不跳。
        try:
            blobs = img.find_blobs([list(c["center_thr"])], area_threshold=10, pixels_threshold=10,
                                   roi=self._roi(img))
        except Exception:
            blobs = img.find_blobs([list(c["center_thr"])], area_threshold=10, pixels_threshold=10)
        sx = sy = sw = n = 0
        x0 = y0 = 99999
        x1 = y1 = 0
        for b in blobs:
            bx, by, bw, bh = self._bbox_from_corners(b.corners())
            cxb, cyb = bx + bw // 2, by + bh // 2
            if cyb < y_lo or cyb > y_hi:
                continue
            wgt = bw * bh                       # 按面积加权
            sx += cxb * wgt
            sy += cyb * wgt
            sw += wgt
            n += 1
            x0, y0 = min(x0, bx), min(y0, by)   # 顺便求整体外接框, 估算靶径
            x1, y1 = max(x1, bx + bw), max(y1, by + bh)
        if sw == 0:
            return None
        cx, cy = sx // sw, sy // sw
        size = max(x1 - x0, y1 - y0)            # 靶整体直径估计
        img.draw_circle(cx, cy, max(6, size // 2), image.COLOR_GREEN, 1)
        return (cx, cy, size)

    def _proc_target(self, img):
        c = self.p.d["target"]
        center = self._find_target_center(img)          # (cx, cy, size) 或 None
        if center:
            img.draw_line(center[0] - 6, center[1], center[0] + 6, center[1], image.COLOR_GREEN, 1)
            img.draw_line(center[0], center[1] - 6, center[0], center[1] + 6, image.COLOR_GREEN, 1)
        laser = self._find_biggest(img, c["laser_thr"], 2, 2, image.COLOR_RED)
        if not (center and laser):
            return proto.pack_target(0, 0, 0, False), "tgt:%s laser:%s" % (bool(center), bool(laser)), None
        lx, ly = laser[0] + laser[2] // 2, laser[1] + laser[3] // 2
        dx_px, dy_px = lx - center[0], ly - center[1]
        # 像素->0.1cm 粗换算(按靶径线性), D6 标定日升级为单应性
        scale = float(c["target_w_cm"]) / max(1, center[2])
        return (proto.pack_target(int(dx_px * scale * 10), int(dy_px * scale * 10), 1, True),
                "d=(%d,%d)px" % (dx_px, dy_px), (dx_px, dy_px))

    def _proc_detect(self, img):
        if not self._ensure_detector():
            return proto.pack_detect(0, 0, 0, 0, False), "no model, see DET_TRAINING_GUIDE"
        conf = float(self.p.d["detect"]["conf"])
        iw, ih = self.detector.input_width(), self.detector.input_height()
        img_in = img.resize(iw, ih) if (img.width() != iw or img.height() != ih) else img
        objs = self.detector.detect(img_in, conf_th=conf, iou_th=0.45)
        if not objs:
            return proto.pack_detect(0, 0, 0, 0, False), "no obj"
        best = max(objs, key=lambda o: o.score)
        sx, sy = img.width() / iw, img.height() / ih
        x, y = int(best.x * sx), int(best.y * sy)
        w, h = int(best.w * sx), int(best.h * sy)
        img.draw_rect(x, y, w, h, image.COLOR_RED, 2)
        label = self.detector.labels[best.class_id] if best.class_id < len(self.detector.labels) else "?"
        img.draw_string(x, max(0, y - 12), "%s %.2f" % (label, best.score), image.COLOR_RED)
        return (proto.pack_detect(best.class_id, x + w // 2, y + h // 2, best.score, True),
                "%s %.2f" % (label, best.score))

    def _proc_circle(self, img):
        c = self.p.d["circle"]
        ds = int(c.get("downscale", 2))         # 降采样倍数: 2=缩到一半跑(计算量1/4), 1=原图
        ds = max(1, min(4, ds))
        y_lo, y_hi = self.UI_TOP, img.height() - self.UI_BOT
        try:
            if ds > 1:
                small = img.resize(img.width() // ds, img.height() // ds)
                circles = small.find_circles(threshold=int(c["threshold"]),
                                             r_min=max(1, int(c["r_min"]) // ds),
                                             r_max=max(2, int(c["r_max"]) // ds))
            else:
                circles = img.find_circles(threshold=int(c["threshold"]),
                                           r_min=int(c["r_min"]), r_max=int(c["r_max"]))
        except Exception as e:
            log("find_circles err:", repr(e))
            return proto.pack_circle(0, 0, 0, False), "circ err"
        best, best_r = None, -1
        for a in circles:
            # 坐标从小图放大回原图
            ax, ay, ar = a.x() * ds, a.y() * ds, a.r() * ds
            if ay < y_lo or ay > y_hi:
                continue
            if ar > best_r:
                best_r, best = ar, (ax, ay, ar)
        if not best:
            return proto.pack_circle(0, 0, 0, False), "no circle"
        cx, cy, r = best
        img.draw_circle(cx, cy, r, image.COLOR_GREEN, 2)
        img.draw_line(cx - 6, cy, cx + 6, cy, image.COLOR_GREEN, 1)
        img.draw_line(cx, cy - 6, cx, cy + 6, image.COLOR_GREEN, 1)
        return proto.pack_circle(cx, cy, r, True), "c=(%d,%d) r=%d ds%d" % (cx, cy, r, ds)

    def _proc_tag(self, img):
        # 先找 AprilTag
        try:
            tags = img.find_apriltags(families=image.ApriltagFamilies.TAG36H11)
        except Exception as e:
            log("find_apriltags err:", repr(e))
            tags = []
        if tags:
            a = tags[0]
            corners = a.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                              corners[(i + 1) % 4][0], corners[(i + 1) % 4][1], image.COLOR_GREEN, 2)
            cx, cy, tid = a.cx(), a.cy(), a.id()
            img.draw_string(cx, cy, "tag:%d" % tid, image.COLOR_YELLOW)
            return proto.pack_tag(tid, cx, cy, True), "tag id=%d (%d,%d)" % (tid, cx, cy)
        # 没有 AprilTag 则找 QR 码
        try:
            qrs = img.find_qrcodes()
        except Exception as e:
            log("find_qrcodes err:", repr(e))
            qrs = []
        if qrs:
            q = qrs[0]
            corners = q.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                              corners[(i + 1) % 4][0], corners[(i + 1) % 4][1], image.COLOR_BLUE, 2)
            cx, cy = q.x() + q.w() // 2, q.y() + q.h() // 2
            payload = q.payload()
            img.draw_string(2, 48, "QR: " + payload[:24], image.COLOR_BLUE)
            self.last_qr = payload          # 存最近一次内容, 供查看
            # QR 内容是字符串, 协议帧用 id=-1 标记是QR(区别于tag), 坐标照发
            return proto.pack_tag(-1, cx, cy, True), "QR: " + payload[:20]
        return proto.pack_tag(0, 0, 0, False), "no tag/qr"

    def _proc_gimbal(self, img):
        """云台视觉自闭环: 把跟踪源收敛到瞄准点。
        Src=0 跟最大色块(原行为); Src=1 跟靶心(打靶题, 复用 TARGET 页调好的检测方法与阈值)。
        瞄准点 = 画面中心 + AimOff(激光器与相机光轴平行安装时的校靶偏移, 默认0=正中心)。"""
        if not self._ensure_gimbal():
            return None, "gimbal:" + self.gim_err
        g = self.p.d["gimbal"]
        src = int(g.get("src", 0)) % 2
        ax = img.width() // 2 + int(g.get("aim_off_x", 0))
        ay = img.height() // 2 + int(g.get("aim_off_y", 0))
        if src == 1:
            t = self._find_target_center(img)           # (cx, cy, size) 或 None
            c = (t[0], t[1]) if t else None
            pkt_lost = proto.pack_target(0, 0, 0, False)
        else:
            pkt_blob, _, c = self._proc_blob(img)
            pkt_lost = pkt_blob                          # blob 的无效帧
        if not c:
            self.gimbal.hold()                           # 丢目标: 保持当前位置, 不甩不漂
            self.lock_n = 0
            self._draw_center_cross(img, aim=(ax, ay))
            # 丢目标也要打日志(记 nan), 否则"没日志"分不清是 Dbg=0 还是根本没检测到靶心
            self._gimb_log(None, None)
            return pkt_lost, "no %s" % ("tgt" if src == 1 else "blob")
        ex, ey = c[0] - ax, c[1] - ay
        self.gimbal.update(ex, ey)
        lk = float(g.get("lock_px", 6))
        # LOCK 只统计"在线的轴": 单轴调试时(如 ID1 烧毁, Axes=2)另一轴的误差永远收不掉,
        # 若还参与判定就永远不会 LOCK, 没法验证判稳逻辑。双轴(Axes=3)时自动恢复完整语义。
        # info 行会同时显示 PIT-only/YAW-only, 所以 LOCK 的含义始终是"所有在线轴已对准", 不会误导。
        okx = abs(ex) <= lk or not getattr(self.gimbal, "use_yaw", True)
        oky = abs(ey) <= lk or not getattr(self.gimbal, "use_pitch", True)
        self.lock_n = self.lock_n + 1 if (okx and oky) else 0
        locked = self.lock_n >= 10
        # Dbg=1: 每帧打印 误差 + 指令角, 用于整定
        self._gimb_log(ex, ey)
        self._draw_center_cross(img, ex, ey, aim=(ax, ay))
        if locked:
            img.draw_string(ax + 12, ay - 24, "LOCK", image.COLOR_GREEN, scale=1.2)
        # Src=1 时按 TARGET 帧发"靶心相对瞄准点偏差(px)", 主控可据此蜂鸣/判稳(见 PROTOCOL.md)
        pkt = proto.pack_target(ex, ey, 0, True) if src == 1 else pkt_blob
        st = self.gimbal.status_str() if hasattr(self.gimbal, "status_str") else ""
        return pkt, "e=(%d,%d)%s%s" % (ex, ey, " LOCK" if locked else "", st)

    # ---------------- UI ----------------
    def _build_ui(self):
        iw, ih = int(self.p.d["global"]["cam_w"]), int(self.p.d["global"]["cam_h"])
        dw, dh = self.disp.width(), self.disp.height()
        self.btns = []
        bw = iw // len(MODE_NAMES)
        for i, name in enumerate(MODE_NAMES):
            self.btns.append(Button(i * bw, 0, bw - 1, 20, name, lambda m=i: self._switch_mode(m)))
        y = ih - 22
        self.btns.append(Button(0, y, 26, 22, "<", lambda: self._param_nav(-1)))
        self.btns.append(Button(27, y, 26, 22, ">", lambda: self._param_nav(1)))
        self.btns.append(Button(54, y, 26, 22, "-", lambda: self._param_adj(-1)))
        self.btns.append(Button(81, y, 26, 22, "+", lambda: self._param_adj(1)))
        self.btns.append(Button(iw - 130, y, 42, 22, "PICK", self._toggle_pick))
        self.btns.append(Button(iw - 86, y, 40, 22, "SV", self._param_save))
        self.btns.append(Button(iw - 44, y, 44, 22, "EXIT", lambda: app.set_exit_flag(True)))
        for b in self.btns:
            b.map_disp(iw, ih, dw, dh)
        self._pressed = False
        self._press_xy = (0, 0)
        # 取色状态
        self.pick_mode = False
        self._drag = False
        self._drag_start = (0, 0)
        self._drag_cur = (0, 0)
        self.pick_info = ""

    def _toggle_pick(self):
        """切换取色模式。开启后手指在画面上框选目标, 自动算阈值写入当前模式。"""
        path = self._pick_target_path()
        if path is None:
            self.pick_info = "此模式不支持取色"
            return
        self.pick_mode = not self.pick_mode
        self._drag = False
        if self.pick_mode:
            self.pick_info = "框选目标(%s)" % path[-1][:3]

    def _gimb_log(self, ex, ey):
        """Dbg=1 时每帧打印一行, 供 tools/analyze_gimblog.py 分析。
        格式: GIMBLOG,帧号,ex,ey,yaw指令角,pitch指令角  (逗号分隔; 丢目标时 ex/ey 记 nan)
        丢目标也照打, 这样"没有 GIMBLOG 输出"就只可能是 Dbg=0, 不会和"没检测到"混淆。"""
        if not int(self.p.d["gimbal"].get("dbg", 0)):
            return
        self.gimb_n = getattr(self, "gimb_n", 0) + 1
        log("GIMBLOG,%d,%s,%s,%.2f,%.2f" % (
            self.gimb_n,
            "nan" if ex is None else int(ex),
            "nan" if ey is None else int(ey),
            getattr(self.gimbal, "ang_yaw", 0.0),
            getattr(self.gimbal, "ang_pitch", 0.0)))

    def _param_list(self):
        return EDITABLE.get(self.mode, [])

    def _param_nav(self, d):
        n = len(self._param_list())
        if n:
            self.param_idx = (self.param_idx + d) % n

    def _param_adj(self, d):
        lst = self._param_list()
        if not lst:
            return
        name, path, step, lo, hi = lst[self.param_idx % len(lst)]
        val = self.p.get(path)
        if name == "Baud":
            i = (BAUDS.index(int(val)) + d) % len(BAUDS) if int(val) in BAUDS else 0
            new = BAUDS[i]
        else:
            new = val + d * step
            new = max(lo, min(hi, new))
            if isinstance(val, int) and float(step) == int(step):
                new = int(new)
            else:
                new = round(float(new), 6)
        self.p.set(path, new)
        # 每次改参数都打一行。banner 只在进模式那一刻打一次, 之后你在触屏上改了什么它不知道
        # (我曾据此误判 Src 还是 0)。这行让日志始终反映【当前真实生效值】。
        # 注意: 改完要按 SV 存盘, 否则重启回默认值。
        log("param: %s = %s%s" % (name, new, "" if self._param_dirty else "   (记得按 SV 存盘)"))
        self._param_dirty = True
        if path[0] == "global" and name in ("ExpUs", "AWBloc", "Gain", "WB_R", "WB_B"):
            self._apply_cam_params()

    def _pick_target_path(self):
        """当前模式下, 取色结果应写入哪个阈值参数。"""
        if self.mode == MODE_LINE:
            return ("line", "thr")
        if self.mode == MODE_BLOB:
            return ("blob", "thr")
        if self.mode == MODE_TARGET:
            # pick_which_idx: 0=靶心 1=激光, 在 TARGET 页 PickWhat 参数切换
            return ("target", "laser_thr") if int(self.p.d["target"].get("pick_which_idx", 0)) == 1 \
                else ("target", "center_thr")
        return None

    def _do_pick(self, img, x0, y0, x1, y1):
        """对图像坐标框 (x0,y0)-(x1,y1) 区域统计 LAB, 算出阈值写入当前模式。
        关键: 框选圆环/小目标时框内难免混入白背景, 故先排除中性色(白/灰/黑)像素,
        只统计真正有彩色的部分, 避免阈值被背景撑大。"""
        path = self._pick_target_path()
        if path is None:
            return "pick: 此模式不支持取色"
        x, y = max(0, min(x0, x1)), max(0, min(y0, y1))
        w, h = abs(x1 - x0), abs(y1 - y0)
        if w < 4 or h < 4:
            return "pick: 框太小"
        w = min(w, img.width() - x)
        h = min(h, img.height() - y)
        k = float(self.p.d.get("pick_k", 2.0))

        def rng(mean, std, lo, hi, pad):
            return max(lo, int(mean - k * std - pad)), min(hi, int(mean + k * std + pad))

        try:
            roi_img = img.crop(x, y, w, h)
            is_line = (self.mode == MODE_LINE)   # 黑线是"暗且无彩色", 不能按彩色筛
            if is_line:
                st = roi_img.get_statistics([[0, 100, -128, 127, -128, 127]])
            else:
                # 先探测框内主色方向: 分别统计"偏红/偏绿/偏蓝/偏黄"像素, 取占比最大的
                # 排除白/灰/黑(A,B 都接近0的中性色)。cnt 越大说明该色相是主目标。
                probes = {
                    "red":   [[0, 100, 15, 127, -128, 127]],
                    "green": [[0, 100, -128, -15, -128, 127]],
                    "blue":  [[0, 100, -128, 127, -128, -15]],
                    "yellow":[[0, 100, -128, 127, 15, 127]],
                }
                best_name, best_cnt, best_st = None, -1, None
                for nm, thr in probes.items():
                    blobs = roi_img.find_blobs(thr, area_threshold=10, pixels_threshold=10)
                    pix = sum(b.pixels() for b in blobs) if blobs else 0
                    if pix > best_cnt:
                        best_cnt, best_name = pix, nm
                        best_st = roi_img.get_statistics(thr)
                if best_cnt <= 0:
                    # 框内没有明显彩色, 退回全区域统计(可能是要取黑/白/灰目标)
                    st = roi_img.get_statistics([[0, 100, -128, 127, -128, 127]])
                else:
                    st = best_st
        except Exception as e:
            log("pick stat err:", repr(e))
            return "pick: 统计失败"

        lmin, lmax = rng(st.l_mean(), st.l_std_dev(), 0, 100, 3)
        amin, amax = rng(st.a_mean(), st.a_std_dev(), -128, 127, 3)
        bmin, bmax = rng(st.b_mean(), st.b_std_dev(), -128, 127, 3)
        thr = [lmin, lmax, amin, amax, bmin, bmax]
        self.p.set(path, thr)
        log("picked", path, "->", thr)
        return "picked %s L%d-%d A%d-%d B%d-%d" % (path[-1][:3], lmin, lmax, amin, amax, bmin, bmax)

    def _param_save(self):
        if self.p.save():
            self._param_dirty = False
            log("param: 已存盘 -> 重启后仍生效")

    def _param_text(self):
        lst = self._param_list()
        if not lst:
            return "-"
        name, path, _, _, _ = lst[self.param_idx % len(lst)]
        v = self.p.get(path)
        return "%s=%s" % (name, ("%.4f" % v) if isinstance(v, float) else v)

    def _handle_touch(self, img):
        x, y, pressed = self.ts.read()
        if self.pick_mode:
            # 取色模式: 手指按下->拖动->松手, 画框并在松手时统计
            ix, iy = image.resize_map_pos_reverse(img.width(), img.height(),
                                                  self.disp.width(), self.disp.height(),
                                                  image.Fit.FIT_CONTAIN, x, y)
            if pressed:
                if not self._drag:
                    self._drag = True
                    self._drag_start = (ix, iy)
                self._drag_cur = (ix, iy)
            elif self._drag:
                self._drag = False
                sx, sy = self._drag_start
                self.pick_info = self._do_pick(img, sx, sy, ix, iy)
                self.pick_mode = False          # 取完自动退出取色模式
            return
        # 普通模式: 按钮点击(松手触发, 防连击)
        if pressed:
            self._pressed = True
            self._press_xy = (x, y)
        elif self._pressed:
            self._pressed = False
            px, py = self._press_xy
            for b in self.btns:
                if b.hit(px, py):
                    try:
                        b.cb()
                    except Exception as e:
                        log("btn err:", e)
                    break

    def _draw_ui(self, img, info):
        for i, b in enumerate(self.btns):
            b.draw(img, active=(i == self.mode and i < len(MODE_NAMES)))
        img.draw_string(2, 24, "fps:%d %s" % (self.fps.val, info), image.COLOR_YELLOW)
        img.draw_string(110, img.height() - 18, self._param_text(), image.COLOR_YELLOW)
        # 取色模式: 画拖拽框 + 提示
        if self.pick_mode:
            img.draw_string(2, 36, "PICK: " + self.pick_info, image.COLOR_GREEN)
            if self._drag:
                sx, sy = self._drag_start
                cx, cy = self._drag_cur
                x, y = min(sx, cx), min(sy, cy)
                img.draw_rect(x, y, abs(cx - sx), abs(cy - sy), image.COLOR_GREEN, 2)
        elif self.pick_info:
            img.draw_string(2, 36, self.pick_info, image.COLOR_GREEN)

    # ---------------- 主循环 ----------------
    def run(self):
        procs = {MODE_LINE: self._proc_line,
                 MODE_DETECT: self._proc_detect, MODE_GIMBAL: self._proc_gimbal,
                 MODE_CIRCLE: self._proc_circle, MODE_TAG: self._proc_tag}
        while not app.need_exit():
            try:
                self._run_one_frame(procs)
            except Exception as e:
                # 最外层兜底: 任何单帧异常只跳过该帧, 打印后继续, 绝不让程序退出
                log("FRAME ERR:", repr(e))
                if self.wd:
                    self.wd.feed()
                time.sleep_ms(20)

        if self.gimbal:
            try:
                self.gimbal.center()
            except Exception:
                pass
        log("exit")

    def _run_one_frame(self, procs):
        try:
            img = self.cam.read()
            if img is None:
                raise RuntimeError("cam read none")
            self.cam_fail = 0
        except Exception as e:
            self.cam_fail += 1
            log("cam read err:", e, self.cam_fail)
            if self.cam_fail >= 10:          # 相机异常自恢复
                self.cam_fail = 0
                try:
                    self._open_camera()
                except Exception as e2:
                    log("cam reopen failed:", e2)
            if self.wd:
                self.wd.feed()
            time.sleep_ms(50)
            return

        self.frame_i += 1
        self._handle_rx()

        info = ""
        pkt = None
        try:
            if self.mode == MODE_BLOB:
                pkt, info, _ = self._proc_blob(img)
            elif self.mode == MODE_TARGET:
                pkt, info, _ = self._proc_target(img)   # target 返回三元组, 丢弃 center
            elif self.mode in procs:
                pkt, info = procs[self.mode](img)
        except Exception as e:
            info = "proc err"
            log("proc err:", repr(e))
        self._send(pkt)

        now = time.ticks_ms()
        if now - self.last_hb >= 1000:       # 1Hz 心跳
            self.last_hb = now
            try:
                self.uart.write(proto.pack_heartbeat(self.mode, self.fps.val))
            except Exception:
                pass

        self._handle_touch(img)
        self.fps.tick()
        self._draw_ui(img, info)
        self.disp.show(img, fit=image.Fit.FIT_CONTAIN)
        if self.wd:
            self.wd.feed()


if __name__ == "__main__":
    VisionApp().run()
