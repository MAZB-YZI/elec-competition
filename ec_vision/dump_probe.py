# -*- coding: utf-8 -*-
"""dump_probe.py v2 — 实时预览 + 全部色块转储 (能一边对准一边看结果)

v1 的死法: 我没建 display.Display() -> 屏幕全黑, 你只能盲拍。
  于是 thr_probe 那份"L上限扫描"数据很可能拍的根本不是靶纸, 而我还拿它推了
  "框碎了""背景块贴边"一堆理论。教训: 探测脚本更要有预览, 否则连拍的是什么都不知道。

本脚本:
  - 屏幕实时显示画面, 你可以对准
  - 屏幕上直接画出所有候选块, 并标出哪个像 A4 黑框(绿) / 背景(红) / 实心图形(黄)
  - 每 2 秒往终端打一次【全部】色块(不筛选)
  - 按屏幕左上角区域 或 等 SAVE_AT 帧后自动存一张原图

存图路径会在终端打印。拉下来:  scp root@<相机IP>:/root/frame.jpg .
"""
import math
from maix import camera, display, image, app, time

CAM_W, CAM_H = 640, 480
L_MAX   = 60          # 黑色 L 上限, 先给个中间值; 看预览里黑框有没有被完整框住再调
SAVE_AT = 60          # 第几帧存原图
OUT     = "/root/frame.jpg"

def shoelace(p):
    s = 0.0
    for i in range(len(p)):
        s += p[i][0]*p[(i+1) % len(p)][1] - p[(i+1) % len(p)][0]*p[i][1]
    return abs(s)/2

def kind(x, y, w, h, A):
    if x <= 1 or y <= 1 or x+w >= CAM_W-2 or y+h >= CAM_H-2:
        return "背景/环境", image.COLOR_RED
    if abs(A - 0.30) < 0.10:
        return "★像黑框", image.COLOR_GREEN
    if A > 0.85:
        return "实心方", image.COLOR_YELLOW
    return "其他", image.COLOR_BLUE

cam = camera.Camera(CAM_W, CAM_H)
disp = display.Display()
print("实时预览已开。把靶纸对准, 看屏幕上:")
print("  绿框 = 像 A4 黑框(A≈0.30)   红框 = 贴边的背景   黄框 = 实心图形   蓝框 = 其他")
print("每 2 秒打一次全部色块。第 %d 帧会存原图到 %s\n" % (SAVE_AT, OUT))

n, last = 0, 0
while not app.need_exit():
    img = cam.read()
    n += 1
    bs = img.find_blobs([[0, L_MAX, -25, 25, -25, 25]], area_threshold=150, pixels_threshold=150)
    rows = []
    for b in bs:
        x, y, w, h = b.rect()
        am = shoelace(b.mini_corners())
        A = b.pixels()/am if am > 1 else 0
        k, col = kind(x, y, w, h, A)
        img.draw_rect(x, y, w, h, col, 2)
        rows.append((w*h, x, y, w, h, b.pixels(), A, k))
    rows.sort(reverse=True)
    ring = [r for r in rows if r[7] == "★像黑框"]
    img.draw_string(4, 4, "L<=%d  %d blobs  %s" % (
        L_MAX, len(bs), ("框: %dx%d" % (ring[0][3], ring[0][4])) if ring else "没有像框的"),
        image.COLOR_WHITE)
    if n == SAVE_AT:
        try:
            img.save(OUT)
            print(">>> 原图已存: %s  (把它拉下来看)\n" % OUT)
        except Exception as e:
            print(">>> 存图失败: %s" % e)
    t = time.ticks_ms()
    if t - last > 2000:
        last = t
        print("--- 帧%d  L<=%d  共%d块 ---" % (n, L_MAX, len(bs)))
        print("  %-5s %-5s %-6s %-6s %-8s %-7s %s" % ("x", "y", "w", "h", "pixels", "A", "判断"))
        for _, x, y, w, h, px, A, k in rows[:6]:
            print("  %-5d %-5d %-6d %-6d %-8d %-7.3f %s" % (x, y, w, h, px, A, k))
        if not ring:
            print("  ★ 没有像黑框的块。检查: 靶纸在画面里吗? L_MAX 要不要调? 距离对吗?")
    disp.show(img)
