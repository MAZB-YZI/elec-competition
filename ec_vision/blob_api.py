# -*- coding: utf-8 -*-
"""blob_api.py — 问出 MaixPy 的 Blob 到底有哪些方法(一次性, 跑完就知道)

不用对准任何东西, 随便拍到点带颜色的就行。
"""
from maix import camera, image

THR = [0, 80, 40, 80, 10, 80]
cam = camera.Camera(320, 240)
for _ in range(30):
    img = cam.read()
    bs = img.find_blobs([THR], area_threshold=100, pixels_threshold=100)
    if not bs:
        continue
    b = bs[0]
    print("=== Blob 的全部方法 ===")
    print([m for m in dir(b) if not m.startswith("_")])
    print("\n=== 逐个调用看返回什么 ===")
    for m in [m for m in dir(b) if not m.startswith("_")]:
        try:
            v = getattr(b, m)
            print("  %-16s -> %s" % (m, str(v() if callable(v) else v)[:60]))
        except Exception as e:
            print("  %-16s -> 调用失败: %s" % (m, str(e)[:40]))
    break
else:
    print("30 帧都没找到色块, 拿个红色东西对着相机再跑")
