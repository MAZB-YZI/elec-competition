# -*- coding: utf-8 -*-
"""shape_test.py — 形状识别 v3 (阈值已按实测标定)

演进史(都是被数据推翻的):
  v1 用 corners() 算填充率 -> 实测发现 corners() 返回【四个极值点】,
     正方形45° 和 三角形 读数完全一样(都≈1.0)。作废。
  v2 改用 mini_corners()(最小外接旋转矩形) + compactness 双特征。
     实测: A 成立, B(compactness) 作废 —— 同一个圆 B 在 0.139~0.723 之间跳了5倍,
     perimeter 被像素锯齿/摩尔纹搞废了。只留 A。
  v3 = 只用 A, 阈值按实测标定。

特征 A = pixels / mini_corners面积       实测(n=63):
        理论      实测范围         均值      偏差
  三角  0.500    0.529~0.563     0.543    +8.6%   (目标小到 ~1200px 时飘到 0.61, 见 MIN_PIX)
  圆    0.785    0.708~0.815     0.744    -5.2%
  方    1.000    0.932~1.008     0.982    -1.8%   ← 跨 rot=0/114/171, 旋转不变性成立

⚠ blob.area() 是【外接框面积】(w*h), 不是像素数! pixels() 才是。
"""
from maix import camera, display, image, app

THR = [0, 80, 40, 80, 10, 80]

def shoelace(pts):
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i+1) % len(pts)]
        s += x1*y2 - x2*y1
    return abs(s)/2.0

# 阈值 = 实测两类之间的中点(不是理论中点 —— 实测三角偏高、圆偏低)
T_TRI_CIR = 0.66      # 三角(max 0.61) 与 圆(min 0.708) 的中点, 间隙仅 0.098 -> 偏窄
T_CIR_SQ  = 0.87      # 圆(max 0.815) 与 方(min 0.932) 的中点, 间隙 0.117 -> 宽松
MIN_PIX   = 1800      # 目标太小时 A 会飘 (2700px->0.54准 2000px->0.545准 1200px->0.60飘)
                      # 崩溃点在 ~1500px。原设 3000 太严, 把 2300~2900px 的好数据全拒了。

MIN_SOLID = 0.92      # 凸性门限。★实测更正: MaixPy 的 solidity 量的是【外轮廓凸不凸】,
                      #   不是【有没有洞】:
                      #     五角星 实测 0.852~0.867 -> 拦得住(外轮廓凹)
                      #     圆环   实测 1.000       -> 拦不住! 外轮廓是个圆, 完全凸
                      #   所以圆环仍会被误判成 TRIANGLE(A≈0.60)。这是已知边界, 补不上。
                      #   好在 2025C/2019G/2021F 考的都是 圆/等边三角形/正方形, 碰不到圆环。
                      #   门限从 0.90 提到 0.92: 五角星实测 0.867, 余量从 0.03 提到 0.05
AR_TOL    = 1.25      # 长宽比容差: >此值即判为"长"的那种(长方形/椭圆)

def aspect(mc):
    """从 mini_corners 四个点算长宽比(长边/短边), 旋转不变"""
    d = lambda p, q: ((p[0]-q[0])**2 + (p[1]-q[1])**2) ** 0.5
    e1, e2 = d(mc[0], mc[1]), d(mc[1], mc[2])
    lo, hi = min(e1, e2), max(e1, e2)
    return hi / max(1e-6, lo)

def classify(a, px, solid, ar):
    """三维判据:
       A(填充率)  -> 三角/圆/四边形
       solidity   -> 非凸(圆环/星形)直接拒判, 这两个 A 落在三角区会被误判
       长宽比      -> 正方形 vs 长方形;  圆 vs 椭圆
    实测依据(n=63 + 14张找茬图, 预测全中):
       三角 0.52~0.54(等边/直角/钝角都一样, 定理)  圆 0.71~0.83  方 0.97~0.99
       圆环 0.60 星形 0.42 -> 都会掉进三角区, 只能靠 solidity 拦
    """
    if px < MIN_PIX:        return "TOO_SMALL"    # 太小时 A 会往上飘, 拒判好过误判
    if solid < MIN_SOLID:   return "NOT_CONVEX"   # 圆环/星形/破损轮廓
    if a < T_TRI_CIR:       return "TRIANGLE"
    if a < T_CIR_SQ:        return "ELLIPSE" if ar > AR_TOL else "CIRCLE"
    return "RECT" if ar > AR_TOL else "SQUARE"

cam = camera.Camera(320, 240)
disp = display.Display()
print("A=填充率(分三/圆/方)  solid=凸性(拦圆环/星形)  长宽比(分正方/长方, 圆/椭圆)")
print("实测基准: 三角 0.53~0.56   圆 0.71~0.82   方 0.93~1.01   (阈值 %.2f / %.2f, 最小 %dpx)\n"
      % (T_TRI_CIR, T_CIR_SQ, MIN_PIX))
print("%-6s %-8s %-8s %-8s %-8s %-8s %s" % ("帧", "A", "solid", "长宽比", "pixels", "rot", "判定"))
n = 0
while not app.need_exit():
    img = cam.read()
    blobs = img.find_blobs([THR], area_threshold=300, pixels_threshold=300)
    best, ba = None, -1
    for b in blobs:
        if b.pixels() > ba:
            ba, best = b.pixels(), b
    if best:
        mc = best.mini_corners()
        am = shoelace(mc)
        A = best.pixels()/am if am > 1 else 0
        sol = best.solidity()
        ar = aspect(mc)
        name = classify(A, best.pixels(), sol, ar)
        n += 1
        if n % 20 == 0:
            print("%-6d %-8.3f %-8.3f %-8.2f %-8d %-8d %s" % (
                n, A, sol, ar, best.pixels(), best.rotation_deg(), name))
        for i in range(4):
            img.draw_line(int(mc[i][0]), int(mc[i][1]), int(mc[(i+1) % 4][0]), int(mc[(i+1) % 4][1]),
                          image.COLOR_GREEN, 2)
        img.draw_string(4, 4, "%s A=%.2f sol=%.2f ar=%.2f" % (name, A, sol, ar), image.COLOR_WHITE)
    disp.show(img)
