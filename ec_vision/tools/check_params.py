# -*- coding: utf-8 -*-
"""check_params.py — 参数表自检(在电脑上跑: python3 tools/check_params.py)

存在的理由: 触屏条目表(EDITABLE)和默认值表(DEFAULTS)是两份独立的东西, 改了一个忘了另一个
不会报错 —— 只会表现为"参数在触屏上翻不到"或"翻到了但改了个不存在的键"。
曾经真的发生过: circ_r/circ_t 加进了 DEFAULTS, 但 EDITABLE 那行没改成功, 于是参数
存在却够不着, 而且当时的脚本还无条件打印了"updated"。所以要有这个自检。
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from params import EDITABLE, DEFAULTS

def get(d, path):
    for p in path:
        d = d[p]
    return d

bad = []
for page, entries in EDITABLE.items():
    seen = set()
    for e in entries:
        name, path = e[0], e[1]
        # 1. path 必须能在 DEFAULTS 里取到
        try:
            v = get(DEFAULTS, path)
        except Exception as ex:
            bad.append("页%s %s: path %s 在 DEFAULTS 里不存在 (%s)" % (page, name, path, ex))
            continue
        # 2. 默认值必须落在 [lo, hi] 内, 否则一进页面就被夹掉
        if len(e) >= 5 and isinstance(v, (int, float)) and not (e[3] <= v <= e[4]):
            bad.append("页%s %s: 默认值 %s 不在范围 [%s,%s]" % (page, name, v, e[3], e[4]))
        # 3. 同一页不能有重名(会翻到两个都叫这个名字的参数)
        if name in seen:
            bad.append("页%s %s: 同页重名" % (page, name))
        seen.add(name)

if bad:
    print("参数表自检失败:")
    for b in bad:
        print("  ✗", b)
    sys.exit(1)
print("参数表自检通过 ✓  (%d 页, 共 %d 个条目)" % (len(EDITABLE), sum(len(v) for v in EDITABLE.values())))
print("\nGIMB(页5) 触屏顺序:")
for i, e in enumerate(EDITABLE[5]):
    print("  按 %2d 次 >  ->  %-8s = %s" % (i, e[0], get(DEFAULTS, e[1])))
