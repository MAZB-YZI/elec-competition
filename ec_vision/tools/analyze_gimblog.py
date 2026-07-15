# -*- coding: utf-8 -*-
"""
analyze_gimblog.py — 把 GIMB 的 Dbg 日志换算成"该设多少 FKp"(在电脑上跑, 不是相机上)

用法:
  1. 触屏 GIMB 页把 Dbg 设为 1, 靶纸放好让它跟踪(有振荡也没关系, 振荡数据反而最好用)
  2. 把串口/终端里的 GIMBLOG 行复制出来存成 log.txt
  3. python3 analyze_gimblog.py log.txt

它会量出来(而不是猜):
  - ppd  每度对应多少像素 —— 由 云台指令角变化 vs 靶心像素变化 回归得到
  - d    环路延迟几帧     —— 由互相关峰值得到
  - 据此算出 FKp 的临界值和推荐值
"""
import sys
import re


def parse(path):
    rows = []
    pat = re.compile(r"GIMBLOG,(-?\d+),(nan|-?\d+),(nan|-?\d+),(-?[\d.]+),(-?[\d.]+)")
    n_nan = 0
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = pat.search(line)
        if not m:
            continue
        if m.group(2) == "nan" or m.group(3) == "nan":
            n_nan += 1          # 丢目标帧: 不能参与回归, 丢弃
            continue
        rows.append((int(m.group(1)), int(m.group(2)), int(m.group(3)),
                     float(m.group(4)), float(m.group(5))))
    if n_nan:
        print("注意: %d 帧丢目标(nan), 已剔除。占比高说明靶心检测不稳, 先回 TARG 页调阈值。" % n_nan)
    return rows


def diff(a):
    return [a[i + 1] - a[i] for i in range(len(a) - 1)]


def corr(a, b):
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    ma, mb = sum(a) / n, sum(b) / n
    da = [x - ma for x in a]
    db = [x - mb for x in b]
    num = sum(x * y for x, y in zip(da, db))
    den = (sum(x * x for x in da) * sum(y * y for y in db)) ** 0.5
    return num / den if den > 1e-9 else 0.0


def main(path):
    rows = parse(path)
    if len(rows) < 30:
        print("只解析到 %d 行 GIMBLOG, 太少。请让它跑 5~10 秒再复制。" % len(rows))
        return
    ex = [r[1] for r in rows]
    ey = [r[2] for r in rows]
    ayaw = [r[3] for r in rows]
    apit = [r[4] for r in rows]
    print("样本 %d 帧" % len(rows))

    for name, e, a in (("YAW/X", ex, ayaw), ("PITCH/Y", ey, apit)):
        if max(a) - min(a) < 0.5:
            print("\n[%s] 指令角基本没动(该轴离线或没误差), 跳过" % name)
            continue
        print("\n" + "=" * 46)
        print("[%s]  误差 %d~%d px   指令角 %.1f~%.1f°" % (name, min(e), max(e), min(a), max(a)))

        # --- 延迟 d: 云台"动了多少度"要过几帧才反映到"误差变了多少像素" ---
        # 相机装在云台上, 云台正向转 -> 靶心像素朝反方向走, 所以相关应为负
        da, de = diff(a), diff(e)
        best_d, best_c = None, 0.0
        print("  延迟扫描(|相关|越大越像):")
        for d in range(0, 9):
            if len(da) - d < 20:
                break
            c = corr(da[:len(da) - d], de[d:])
            mark = ""
            if abs(c) > abs(best_c):
                best_c, best_d = c, d
                mark = "  ←最强"
            print("    d=%d帧  相关=%+.2f%s" % (d, c, mark))

        # --- ppd: 由 Δ误差(px) / Δ指令角(°) 回归 ---
        x = da[:len(da) - best_d]
        y = de[best_d:]
        n = min(len(x), len(y))
        x, y = x[:n], y[:n]
        sxx = sum(v * v for v in x)
        sxy = sum(u * v for u, v in zip(x, y))
        if sxx < 1e-6:
            print("  指令角变化太小, 无法回归 ppd")
            continue
        slope = sxy / sxx                      # px per degree(带符号)
        ppd = abs(slope)
        print("  → 实测 ppd = %.2f px/度   (斜率 %+.2f, 负号=装在云台上, 正常)" % (ppd, slope))
        print("  → 实测环路延迟 d = %d 帧" % best_d)
        if ppd < 0.5:
            print("  ⚠ ppd 过小: 相机可能【没装在云台上】, 云台转了但画面不动!")
            continue

        # --- 由 ppd 和 d 反推 FKp 临界值 ---
        import numpy as np
        def zmax(kp):
            d = max(best_d, 1)
            c = np.zeros(d + 2)
            c[0] = 1.0
            c[1] = -1.0
            c[d] += kp * ppd
            return max(abs(r) for r in np.roots(c))
        lo, hi = 1e-4, 2.0
        for _ in range(60):
            mid = (lo + hi) / 2
            if zmax(mid) < 1.0:
                lo = mid
            else:
                hi = mid
        print("  → FKp 临界值 = %.3f (超过必等幅/发散)" % lo)
        print("  → 推荐 FKp = %.3f  (临界的 40%%, |z|=%.2f 每帧衰减)" % (lo * 0.4, zmax(lo * 0.4)))
        print("     保守 FKp = %.3f  (临界的 25%%, 最稳)" % (lo * 0.25))

        # --- 振荡周期(有振荡才有意义) ---
        sign = [1 if v > 0 else -1 for v in e]
        cross = sum(1 for i in range(len(sign) - 1) if sign[i] != sign[i + 1])
        if cross >= 4:
            per = 2.0 * (len(sign) - 1) / cross
            print("  → 观测到过零 %d 次 → 振荡周期约 %.1f 帧" % (cross, per))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "log.txt")
