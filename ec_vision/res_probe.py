# -*- coding: utf-8 -*-
"""res_probe.py — 探出 find_rects / find_blobs 各自能撑到多大分辨率(一次性)

起因: rect_test.py 在 1280x720 跑 find_rects 直接
      "MemoryError: Out of fast Frame Buffer Stack Memory" + 段错误。
      但 2025C 的精度要求又需要高分辨率(A4高@200cm: 320宽只有40px, 1%=0.4px 做不到)。
      => 必须知道两个算法各自的天花板在哪, 才能定方案。

不用对准任何东西, 跑完看表。
"""
from maix import camera, image, app

RES = [(320, 240), (640, 480), (800, 600), (960, 540), (1280, 720), (1920, 1080)]
BLACK = [0, 35, -20, 20, -20, 20]      # 黑色 LAB 阈值(找黑框用)

print("%-12s %-16s %-16s %s" % ("分辨率", "find_blobs", "find_rects", "备注"))
print("-"*64)
for w, h in RES:
    try:
        cam = camera.Camera(w, h)
    except Exception as e:
        print("%-12s %s" % ("%dx%d" % (w, h), "相机开不了: %s" % str(e)[:40]))
        continue
    for _ in range(5):
        img = cam.read()                       # 丢几帧等稳定
    rb = rr = "?"
    try:
        bs = img.find_blobs([BLACK], area_threshold=500, pixels_threshold=500)
        rb = "OK (%d个)" % len(bs)
    except Exception as e:
        rb = "✗ " + str(e)[:24]
    try:
        rs = img.find_rects(threshold=10000)
        rr = "OK (%d个)" % len(rs)
    except Exception as e:
        rr = "✗ " + str(e)[:24]
    print("%-12s %-16s %-16s" % ("%dx%d" % (w, h), rb, rr))
    del cam
print("""
读法:
  find_blobs 能撑到多大 -> 决定"黑框当色块找"这条路的精度上限
  find_rects 能撑到多大 -> 决定还要不要用它 (它有外沿/内沿歧义, 本来就不如色块法)
★ 若某一行直接段错误退出, 那一档就是天花板, 上一档才是能用的。
""")
