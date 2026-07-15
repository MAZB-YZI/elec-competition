# -*- coding: utf-8 -*-
"""rect_test.py v2 — A4 黑框测距 (2025 C 题核心)

v1 的死法: 用 find_rects @1280x720 -> "MemoryError: Out of fast Frame Buffer Stack Memory"
           + 段错误。实测天花板:
              分辨率   find_blobs   find_rects
              320      OK           OK
              640      OK           ✗ 静默失败(C层打MemoryError到stderr, 不抛Python异常!)
              800      段错误        段错误
           => 只能 640, 且只能用 find_blobs。

v2 = 把黑框当【色块】找, 不用 find_rects。反而更好:
   ① mini_corners() 直接给外沿四角 -> 【外沿/内沿的歧义不存在了】
      (v1 那个坑很阴: 认错边沿的话 d 居然还是对的, 但边长 x 会系统性错 23%)
   ② A = 黑框像素/mini面积 = 186.8/623.7 = 0.30, 独一无二的指纹, 不会和实心图形混
   ③ 白送 rotation_deg(), 可判物面倾斜(发挥(4)要转 30~60°)

精度(640, f≈539):
   距离    A4高    单帧±1px误差
   100cm  160px  ±0.6cm    全过
   150cm  107px  ±1.4cm    全过
   200cm   80px  ±2.5cm    基本(≤5cm)过, 发挥(≤2cm)差一点
★ 题目给 5 秒 -> 多帧平均, 随机噪声按 √N 降: 平均 4 帧 -> ±1.25cm, 发挥也过。
  但这只对【随机】误差有效。系统性偏差(阈值啃边)平均多少帧都没用, 只能靠标定吸收,
  所以标定必须在和测量【同样的距离范围】做。

用法:
  1. 全屏打开 c_1_frame_empty.png, 【用尺子量黑框外沿的宽】-> 填 W_OUT (屏幕上不是21cm!)
  2. KNOWN_D 填卷尺量的真实距离, F_PX 留 0 -> 标定模式
  3. 记下 f, 填进 F_PX -> 测量模式, 把靶挪到别的距离验证
"""
import math
from maix import camera, display, image, app

CAM_W, CAM_H = 640, 480        # ★ 800 会段错误, 这是天花板
BLACK   = [0, 35, -20, 20, -20, 20]    # 黑框的 LAB 阈值, 找不到就放宽 L 上限
KNOWN_D = 100.0                # 标定: 卷尺量的真实距离(cm)
F_PX    = 0                    # 标定完填这里, 填了就进测量模式
W_OUT   = 21.0                 # ★ 黑框外沿宽(cm)。屏幕显示时改成你【量出来】的值!
H_OUT   = W_OUT * 29.7/21.0    # 长宽比是图自带的, 不用量
AVG_N   = 8                    # 多帧平均帧数(5秒预算够, 白捡 √8=2.8 倍精度)
A_RING  = 0.30                 # 黑框指纹: 框面积/外沿面积。实心图形是 0.5~1.0
                               # ★ 这个值只对【线宽2cm的A4框】成立。屏幕上按比例画的框, 比例一样, 所以通用。

F_TH = 4.7*180/math.pi * CAM_W/320.0
print("相机 %dx%d   理论 f≈%.0f px (由实测 ppd=4.7@320宽 缩放, 仅对照)" % (CAM_W, CAM_H, F_TH))
print("模式: %s   平均 %d 帧\n" % ("标定 KNOWN_D=%.0fcm" % KNOWN_D if F_PX <= 0 else "测量 F_PX=%.0f" % F_PX, AVG_N))

def shoelace(p):
    s = 0.0
    for i in range(len(p)):
        s += p[i][0]*p[(i+1) % len(p)][1] - p[(i+1) % len(p)][0]*p[i][1]
    return abs(s)/2

def edges(mc):
    d = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])
    e = [d(mc[i], mc[(i+1) % 4]) for i in range(4)]
    a, b = (e[0]+e[2])/2, (e[1]+e[3])/2
    return min(a, b), max(a, b)

cam = camera.Camera(CAM_W, CAM_H)
disp = display.Display()
buf = []
if F_PX <= 0:
    print("%-6s %-9s %-9s %-7s %-10s %s" % ("帧", "短边px", "长边px", "A", "f(用宽)", "f(用高)"))
else:
    print("%-6s %-9s %-9s %-10s %-10s %s" % ("帧", "短边px", "长边px", "d(宽)", "d(高)", "d(%d帧均)" % AVG_N))
n = 0
while not app.need_exit():
    img = cam.read()
    blobs = img.find_blobs([BLACK], area_threshold=800, pixels_threshold=800)
    # ★ v2.1 修两个bug(实测打脸):
    #   bug1: 原来"先按面积挑最大, 挑完才检查像不像框" -> 铺满画面的背景黑块永远最大, 永远选它。
    #         实测拿到 479x639 = 【整张图】。改成: 只在【像框的候选】里挑最大。
    #   bug2: 贴边的块必是背景。A4 框应完整在视野内 -> 碰到画面边缘的直接扔。
    #         (实测那个背景块 A=0.38~0.42, 正好落进我原来 |A-0.30|<0.12 的窗口, 窗口太宽了)
    best, ba, rejected = None, -1, ""
    for b in blobs:
        x, y, w, h = b.rect()
        if x <= 1 or y <= 1 or x+w >= CAM_W-2 or y+h >= CAM_H-2:
            rejected = "贴边(背景)"          # 碰到画面边缘 = 环境, 不是靶
            continue
        mc = b.mini_corners()
        am = shoelace(mc)
        if am < 1:
            continue
        A = b.pixels()/am
        if abs(A - A_RING) > 0.10:            # 窗口从 0.12 收到 0.10
            rejected = "A=%.2f 非框" % A
            continue
        if am > ba:
            ba, best = am, b
    if best:
        mc = best.mini_corners()
        am = shoelace(mc)
        A = best.pixels()/am if am > 1 else 0
        s, l = edges(mc)
        for i in range(4):
            img.draw_line(int(mc[i][0]), int(mc[i][1]), int(mc[(i+1) % 4][0]), int(mc[(i+1) % 4][1]),
                          image.COLOR_GREEN, 2)
        n += 1
        if F_PX <= 0:
            fw, fh = s*KNOWN_D/W_OUT, l*KNOWN_D/H_OUT
            if n % 15 == 0:
                print("%-6d %-9.1f %-9.1f %-7.3f %-10.0f %.0f" % (n, s, l, A, fw, fh))
            img.draw_string(4, 4, "CAL A=%.2f f=%.0f/%.0f (理论%.0f)" % (A, fw, fh, F_TH), image.COLOR_WHITE)
        else:
            dw, dh = F_PX*W_OUT/max(1, s), F_PX*H_OUT/max(1, l)
            buf.append(dh)
            if len(buf) > AVG_N:
                buf.pop(0)
            davg = sum(buf)/len(buf)
            if n % 15 == 0:
                print("%-6d %-9.1f %-9.1f %-10.1f %-10.1f %.2f  (rot %d°)" % (
                    n, s, l, dw, dh, davg, best.rotation_deg()))
            img.draw_string(4, 4, "D=%.1fcm (%d帧均)  单帧%.1f" % (davg, len(buf), dh), image.COLOR_WHITE)
    else:
        img.draw_string(4, 4, "无候选  最近一个被拒: %s" % (rejected or "没找到任何黑块"),
                        image.COLOR_RED)
    disp.show(img)
