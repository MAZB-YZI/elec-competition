# 通信问题排查记录

## 问题1：FOLLOW 模式传不过去

**现象：** A 板 OLED 只能显示 Max/Min，Follow 切不回来。

**原因：** FOLLOW+D2off 时 flags=0x00，checksum=0xBB 与帧头碰撞，A 板状态机误判为新帧起始，解析错乱。

**解法：** B 板发包加 seq 字节（每次递增），`checksum = 0xBB ^ seq ^ flags`，seq 每次不同，checksum 永远不会固定等于 0xBB。

---

## 问题2：D1 状态无法同步到 B 板

**现象：** A 板 OLED 显示 D1:ON，B 板 OLED 始终显示 D1:OFF。

**原因：** A 板发包 flags 里只有 bit0（D2 toggle），没有 D1 状态。

**解法：** A 板 flags bit1 加入 D1 状态，B 板解析 `(flags >> 1) & 0x01`。

---

## 问题3：K1 长按 D2 爆闪

**现象：** 按住 K1 超过 850ms，D2 反复翻转。

**原因：** `btn_long_sent` 被 `Comm_Send` 清零后，按键还按着，`Button_Tick` 又触发长按。

**解法：** 加 `btn_long_done` 标志，整个按下周期只允许触发一次长按，松开后才清零。

---

## 问题4：上电 PWM 占空比 10% 而非 50%

**现象：** B 板上电后 PWM 显示 10%，不是 50%。

**原因：** `memset` 把 `vi_voltage` 清零为 0，`UpdatePWM` 每 1ms 调用一次，第一次就用 `vi_voltage=0` 算出 10% 占空比，覆盖了初始值。

**解法：** `AppB_Init` 里 `memset` 后加 `g_state.vi_voltage = 1.65f`，`AppA_Init` 里加 `g_state.vi_instant = 1.65f`，上电 PWM 从 50% 开始，通信建立后用真实值覆盖。

---

## 问题5：断一根线只有一边 LOST

**现象：** 断 A→B 线，B 板显示 LOST，A 板显示 OK。

**原因：** 原始代码只靠超时检测（收到对方包就刷新时间戳），发送方不知道对方收不到自己的包。

**尝试1（flags bit4 报告状态）：** 两边在 flags 里加 bit4 报告自己的 comm_state。**失败：** 死锁——断线后两边都报 LOST，复联后收到对方包看到 bit4=1 又变 LOST，永远出不来。

**尝试2（no_reply_cnt 计数器）：** 每次发包+1，收到回包清零，超过 20 判 LOST。**失败：** 两板全双工独立发包，B 板一直在发，A 板一直在收，计数器一直被清零，检测不了断 A→B 线的情况。

**最终方案（seq 确认机制）：**
- B 板回包带上"我最后收到你的第几号包"（`last_a_seq`）
- A 板对比自己发出的 `tx_seq` 和 B 板确认的 `b_confirmed_seq`，差距超过阈值说明 B 板没收到
- 两边对称实现，A→B 包也带 `last_b_seq`
- 阈值设为 15（15×50ms = 750ms），超时设为 800ms，都在 1 秒以内

**中间踩的坑：**
- 上电时 `b_confirmed_seq=0`，`tx_seq` 涨到 20 立刻误判 LOST → 加 `comm_first_rx` 保护
- 阈值太小（20）复联时 seq 还没追上就误判 → 改成 40，最终优化到 15

---

## 最终协议

**A→B 包（7 字节）：** `[0xAA, a_seq, flags, vi_h, vi_l, last_b_seq, checksum]`

**B→A 包（5 字节）：** `[0xBB, b_seq, flags, last_a_seq, checksum]`

**断线检测：** 超时 800ms 或 seq 差值 >15，任一条件触发即 LOST。
