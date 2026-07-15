# ec_vision — 电赛视觉框架(MaixCAM2 / MaixPy v4)

一套"任何控制题即插即用"的比赛级骨架:**8 模式状态机** + 触屏调参(JSON 持久化)+
框选取色 + MSPM0 串口协议 + 看门狗/相机自恢复 + F32C 云台直驱 + PC 主控模拟器。

设计哲学:**广度优先**。不押某一年的题,把常见视觉基元(循迹/色块/靶心/圆/标签/AI)全部铺开,
比赛当天按题目组合调用。**传统视觉优先,AI 只在必要时用。**

---

## 一、连接 MaixCAM2

| 项 | 值 |
|---|---|
| 网络 | 电脑必须连 **robocup** 无线网络 |
| 相机 IP | **192.168.66.62** |
| SSH | `ssh root@192.168.66.62` |

**分工(重要,别搞反):**

- **文件管理 → 用 MaixVision**:连板、传文件、改代码、看画面预览,都在 IDE 里做。
- **运行代码 → 用 SSH 终端**:
  ```bash
  ssh root@192.168.66.62
  cd /root/ec_vision
  python main.py
  ```

**为什么运行要走 SSH:MaixVision 的终端会截断日志。**
GIMB 整定时 `Dbg=1` 每帧打一行 `GIMBLOG`,几秒就上百行 —— IDE 里刷没了就没法回溯,
`tools/analyze_gimblog.py` 也拿不到完整数据。SSH 终端可以完整滚动、可以重定向到文件:

```bash
python main.py 2>&1 | tee /root/gimb.log
```

拉日志/图片到电脑:

```bash
scp root@192.168.66.62:/root/gimb.log .
scp root@192.168.66.62:/root/frame.jpg .
```

---

## 二、快速上手

1. MaixVision 连板,把整个 `ec_vision` 文件夹作为项目打开(或 `scp` 到 `/root/ec_vision`)。
2. SSH 终端跑 `python main.py`。
3. 屏幕**顶栏 8 个模式按钮**,**底栏**:`<` `>` 换参数、`-` `+` 调值、`PICK` 取色、`SV` 存盘、`EXIT` 退出。
4. 桌上放张黑线纸,点 `LINE`:应看到绿线叠加,左上角显示 err/angle。
5. 协议闭环验证:USB-TTL 插电脑(**MaixCAM2 走 B0/B1,`/dev/ttyS2`**,共地),
   `pip install pyserial` 后 `python host_sim/pc_host_sim.py COM5`,输入 `m 1` ——
   板子应切模式并回心跳。**到这一步协议闭环已通,队友照 `PROTOCOL.md` 接即可。**

> ⚠️ **改完参数一定要按 `SV`**,否则重启回默认值。参数存在 `/root/ec_vision_params.json`,掉电保留。
> 该文件是**设备本地标定结果,不进 git**。

---

## 三、八个模式

| # | 名称 | 干什么 | 输出帧 |
|---|---|---|---|
| 0 | `IDLE` | 空闲,调全局相机/串口参数 | 仅心跳 |
| 1 | `LINE` | 循迹:找黑线,给横向偏差 + 倾角 | `0x01` |
| 2 | `BLOB` | 色块/激光点:找最大色块 | `0x02` |
| 3 | `TARG` | 靶纸:靶心 + 激光点,给相对偏差 | `0x03` |
| 4 | `DET` | AI 识别(YOLO,MaixHub 训练) | `0x04` |
| 5 | `GIMB` | 云台视觉自闭环 | `0x03`/`0x02` |
| 6 | `CIRC` | 圆检测 | `0x05` |
| 7 | `TG/QR` | AprilTag(TAG36H11)+ 二维码,先找 tag 找不到再找 QR | `0x06` |

**关键细节:**

- **TARG** 三种靶心检测法,`Method` 切换:`0`=质心(默认,完整圆环最稳)、`1`=最大色块、`2`=圆检测(圆环残缺时用)。
  `TgtWcm` 填靶纸真实宽度(cm)后 unit 自动变 0.1cm(线性粗换算,标定日换单应性)。
- **GIMB** 的 `Src` 是**两个维度的组合**(目标点怎么来 / 瞄准点怎么来):

  | Src | 目标点 | 瞄准点 |
  |---|---|---|
  | 0 | 最大色块 | 画面中心 + AimOff |
  | 1 | 靶心 | 画面中心 + AimOff |
  | 2 | 靶心 | **实测激光光斑** ★ 打靶推荐 |
  | 3 | 靶心 + 圆上动点 | **实测激光光斑** ★ 画圆 |

  Src≥2 用实测光斑而非常数偏移:激光与光轴有高度差 b,光斑像素偏移 = f·b/d,**随距离变**
  (b=3cm 时 d=50cm 偏 16px、d=150cm 偏 5px)。实测光斑则误差是两个像素坐标相减,与距离/畸变/装歪全无关。

- **TG/QR** 无可调参数(检测器自带阈值)。
- **PICK 取色**:点 `PICK` 后在画面上手指框选目标,自动统计 LAB 写入当前模式阈值。
  只在 LINE / BLOB / TARG 可用。**框内会自动排除中性色(白/灰/黑)像素**,
  所以框圆环时混进白背景也不会把阈值撑大。TARG 页用 `PickWhat` 切"取靶心色"还是"取激光色"。

---

## 四、参数速查

参数按模式分页,底栏 `<` `>` 翻页。以下是每页的条目。

### IDLE(全局)

| 参数 | 作用 | 备注 |
|---|---|---|
| `ExpUs` | 曝光(μs) | **0=自动(默认)**。手动曝光会触发手动白平衡→发绿,只在压暗背景抓激光点时才开 |
| `Gain` | 增益 | 仅 `ExpUs>0` 时生效 |
| `WB_R`/`WB_B` | 手动白平衡红/蓝增益 | 手动曝光时自动套用官方默认值 |
| `AWBloc` | 1=锁白平衡 | |
| `Fps` | 帧率 30~90 | **改后需重启程序** |
| `Baud` | 串口波特率 | 115200/230400/460800/921600,**改后需重启** |

隐藏项(只能改 JSON):`wdt_ms`(0=关看门狗,比赛设 5000)、`send_div`(结果帧分频)、`cam_w`/`cam_h`。

### LINE / BLOB

`Lmin Lmax Amin Amax Bmin Bmax` — LAB 六值阈值,配合 `PICK` 用。
`AreaTh` 面积阈值,BLOB 另有 `PixTh` 像素数阈值。

### TARG

| 参数 | 作用 |
|---|---|
| `TgtWcm` | 靶纸真实宽度(cm),用于 px→cm 换算 |
| `Method` | 0=质心 1=色块 2=圆 |
| `PickWhat` | PICK 取色目标:0=靶心 1=激光 |
| `CtrArea` | 靶心最小面积 |
| `CLmin…CBmax` | 靶心 LAB 阈值(默认偏红) |
| `LLmin…LBmax` | 激光 LAB 阈值 |

### DET

`Conf` 置信度阈值。模型路径改 `params.py` 里 `detect.model`(默认 `/root/models/test.mud`)
或直接编辑 JSON。训练流程见 `DET_TRAINING_GUIDE.md`。

### CIRC

`CircTh` 累加器阈值、`Rmin`/`Rmax` 半径范围、`DownS` 降采样倍数(**2=缩一半跑,计算量 1/4**)。

> `find_circles` 很慢,`DownS=1` 会卡 UI。默认 2。

### GIMB

**结构参数(改后重启生效)**

| 参数 | 作用 |
|---|---|
| `Drv` | **1=F32C 无刷总线电机(实际硬件)** / 0=PWM 舵机(备用) |
| `Axes` | 在线轴:3=双轴 1=仅 YAW 2=仅 PITCH。**电机拆了/烧了必须设对,否则驱动会去戳死地址** |
| `PosMd` | F32C 位置模式:3=多圈直通(手册推荐高频改目标) 1=T型(官方例程)。直通发抽就切 1 |

**F32C 视觉外环**

| 参数 | 作用 |
|---|---|
| `FKp` | ⚠️ **实际是积分增益 I** —— `update()` 里角度是累加的,`kp` 乘的是误差累加和 |
| `FKd` | ⚠️ **实际是比例增益 P** —— 乘的是当前误差 |
| `FSpd` | 位置模式限速(RPM),默认 30 |
| `FStep` | 单帧角度步进上限(度) |
| `YawLim`/`PitLim` | **软件限位(度),必设!** 多圈电机会无限转,不限位会绞断相机排线 |

> **这是 PI 环不是 PD 环。振荡先降 `FKp`,不是加 `FKd`。**
> 默认 0.02/0.0 对 1~4 帧的任意环路延迟都稳(|z|≤0.90),先跑通再往上加。

**通用**

| 参数 | 作用 |
|---|---|
| `Src` | 目标/瞄准点组合,见上表 |
| `CircR`/`CircT` | Src=3 画圆半径(px)/ 周期(秒) |
| `InvX`/`InvY` | 方向反了就切 |
| `DeadPx` | 死区(px),误差小于此值不动,消抖 |
| `LockPx` | 连续 10 帧误差小于此值 → 屏显 `LOCK` |
| `AimOfX`/`AimOfY` | 瞄准点偏移(px),Src<2 时的校靶量 |
| `Dbg` | **1=每帧打印 `GIMBLOG`**,配合 SSH 终端 + `tools/analyze_gimblog.py` 整定 |
| `Kp`/`Kd`/`MaxStep` | 仅 PWM 舵机(Drv=0)用 |

> `LOCK` 只统计**在线的轴** —— 单轴调试时另一轴误差永远收不掉,若参与判定就永远不会 LOCK。
> 双轴时自动恢复完整语义。

---

## 五、文件清单

### 框架本体

| 文件 | 作用 |
|---|---|
| `main.py` | 主程序:状态机、8 模式、触屏 UI、PICK 取色、协议收发、WDT、自恢复 |
| `params.py` | 默认值 + `/root/ec_vision_params.json` 持久化 + 触屏参数注册表 |
| `proto.py` | 串口协议编解码(板端) |
| `f32c.py` | **云台主驱动**:WHEELTEC F32C 无刷总线电机(UART1 / A30/A31 / `/dev/ttyS1`) |
| `gimbal.py` | 云台备用驱动:PWM 舵机(Drv=0),增量式 PD |
| `app.yaml` | 打包成 app 用(开机自启前置) |

### 文档

| 文件 | 作用 |
|---|---|
| `PROTOCOL.md` | **视觉↔主控协议契约,给 MSPM0 队友** |
| `GIMBAL_WIRING.md` | 云台接线、联调顺序、增益整定 |
| `DET_TRAINING_GUIDE.md` | MaixHub 云训练 → 部署 |
| `EXTRACT_FRAMES.md` | 抽帧工具说明 |
| `VERIFY_CHECKLIST.md` | 验收清单 |

### 测试 / 诊断脚本(独立运行,不依赖框架)

| 文件 | 作用 |
|---|---|
| `diag.py` | 视觉诊断:分辨率/曝光/各 find 函数带不带 roi 是否崩,逐个隔离 |
| `f32c_test.py` / `f32c_diag.py` | 云台隔离测试(**先于框架跑**,见 `GIMBAL_WIRING.md`) |
| `servo_test.py` | PWM 舵机隔离测试 |
| `rect_test.py` | A4 黑框测距(2025C 核心) |
| `shape_test.py` | 形状识别,阈值已按实测标定 |
| `ocr_test.py` | PaddleOCR 数字识别隔离验证 |
| `res_probe.py` | 探 `find_rects`/`find_blobs` 各自的分辨率天花板 |
| `dump_probe.py` | 实时预览 + 全色块转储 |
| `blob_api.py` | 问出 Blob 对象有哪些方法 |

### 工具

| 文件 | 跑在哪 | 作用 |
|---|---|---|
| `host_sim/pc_host_sim.py` | 电脑 | 冒充主控,不等小车先打通协议闭环 |
| `mspm0_ref/vision_uart.c` | — | 给队友的 C 解析器,可直接嵌入 MSPM0 工程 |
| `tools/analyze_gimblog.py` | 电脑 | 分析 `GIMBLOG` 日志,整定增益 |
| `tools/check_params.py` | 电脑 | 参数表自检:`EDITABLE` 和 `DEFAULTS` 对不上会漏 |
| `tools/collect_data.py` | 相机 | 采 AI 训练数据 |
| `extract_frames.py` | 电脑 | 视频抽帧生成数据集 |

> AI 数据流水线:`collect_data.py` 采集 → `extract_frames.py` 抽帧 → MaixHub 训练(`DET_TRAINING_GUIDE.md`)→ DET 模式。

---

## 六、硬件约束(踩过坑的,别再踩)

| 约束 | 说明 |
|---|---|
| **`find_rects` 绝不能带 `roi`** | 确认段错误(`diag.py` 隔离验证)。不带 `roi` 正常。`find_blobs`/`get_regression` 带 `roi` 安全 |
| **`find_rects` 分辨率天花板 = 320** | 640 静默失败(C 层打 MemoryError 到 stderr,**不抛 Python 异常**),800 段错误。用 `find_blobs` 代替 |
| **UART2 = B0/B1 = `/dev/ttyS2`** | 丝印 U2R/U2T。主控通信走这条。UART0 是系统打印,勿占 |
| **UART1 = A30/A31 = `/dev/ttyS1`** | 云台 F32C 走这条。**与 PWM6/PWM7 复用 → F32C 和 PWM 舵机方案互斥,不能热切** |
| **`find_circles` 慢** | 必须 2x 降采样,否则 UI 冻结 |
| **相机自动曝光 ~40ms 正常** | 手动曝光会造成绿色偏色、画质更差,**别开** |
| **镜头对焦靠转镜筒** | 不是拧安装孔的螺丝 |
| **电机 12V 独立供电,与板共地** | 12V 绝不可接 MaixCAM2 任何引脚。无防反接保护,接反烧电机,不可热插拔 |

**云台当前状态:** `ID1(X/YAW)已被 60V 烧毁`(电源没调压)。它挂总线上被请求反馈时会拉爆相机重启,
**已物理拔除**。`Axes=2`(仅 PITCH/ID2),母线 10.96V,ID2 健康。**新电机到货后 `Axes` 改回 3。**

---

## 七、封版:开机自启

1. MaixVision:项目 → 打包/安装 APP(用 `app.yaml`,id=`ec_vision`)。
2. 板上执行 `echo ec_vision > /maixapp/auto_start.txt && sync`。
3. 断电重启 **10 连测**:上电→自动运行→串口有心跳,**10/10 才算过**。
4. 同时设 `global.wdt_ms = 5000`、`detect.preload = 1`(先加载模型再喂狗)。

---

## 八、待办

- [ ] **`0x82 SET_PHASE` 下行**:画圆相位必须由主控给(只有它知道车跑到哪),现在用本地计时只够静态验证。见 `PROTOCOL.md §1.5`
- [ ] **标定日**:`_proc_target()` 线性换算 → `cv2.findHomography` 四点标定,输出真实物理坐标
- [ ] 新电机到货 → `Axes` 改回 3
