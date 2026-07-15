/*
 * vision_uart.c — MSPM0 端视觉协议参考实现(与 PROTOCOL.md 逐字节对应)
 * 用法:
 *   1) 在 UART RX 中断里对每个字节调用 vision_feed_byte(b);
 *   2) 解析成功会回调 vision_on_frame(cmd, payload, len), 按需改写;
 *   3) 发送模式切换: vision_send_set_mode(uart_send_fn, VMODE_TARGET);
 * 电气: 3.3V TTL, 板 A16(TX)->MCU RX, 板 A17(RX)<-MCU TX, 共地。默认 115200-8N1。
 */
#include <stdint.h>
#include <string.h>

#define V_HEAD0 0xAA
#define V_HEAD1 0x55
#define V_MAX_PAYLOAD 64

/* 视觉 -> 主控 */
#define VCMD_LINE      0x01  /* int16 err_x; int16 angle_x10; uint8 valid   */
#define VCMD_BLOB      0x02  /* int16 cx,cy,w,h; uint8 valid                */
#define VCMD_TARGET    0x03  /* int16 dx,dy; uint8 unit(0px,1=0.1cm); valid */
#define VCMD_DETECT    0x04  /* uint8 cls; int16 cx,cy; uint8 score; valid  */
#define VCMD_HEARTBEAT 0x0F  /* uint8 mode; uint8 fps                       */
/* 主控 -> 视觉 */
#define VCMD_SET_MODE  0x80  /* uint8 mode */
#define VCMD_PING      0x81

enum { VMODE_IDLE = 0, VMODE_LINE, VMODE_BLOB, VMODE_TARGET, VMODE_DETECT, VMODE_GIMBAL };

/* ------------ 小端取数(协议固定小端) ------------ */
static inline int16_t v_i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* ------------ 应用层回调: 按需改成你的逻辑 ------------ */
extern volatile int16_t g_line_err, g_line_angle_x10;
extern volatile uint8_t g_line_valid;

void vision_on_frame(uint8_t cmd, const uint8_t *pl, uint8_t len)
{
    switch (cmd) {
    case VCMD_LINE:
        if (len >= 5) {
            g_line_err      = v_i16(pl);
            g_line_angle_x10 = v_i16(pl + 2);
            g_line_valid    = pl[4];
        }
        break;
    case VCMD_TARGET:
        /* int16 dx=v_i16(pl), dy=v_i16(pl+2); uint8 unit=pl[4], valid=pl[5]; */
        break;
    case VCMD_BLOB:
    case VCMD_DETECT:
    case VCMD_HEARTBEAT:
    default:
        break;
    }
}

/* ------------ 逐字节状态机 ------------ */
void vision_feed_byte(uint8_t b)
{
    static uint8_t st = 0, ln = 0, cmd = 0, idx = 0;
    static uint8_t buf[V_MAX_PAYLOAD];
    static uint16_t sum = 0;

    switch (st) {
    case 0: st = (b == V_HEAD0) ? 1 : 0; break;
    case 1: st = (b == V_HEAD1) ? 2 : ((b == V_HEAD0) ? 1 : 0); break;
    case 2:
        if (b > V_MAX_PAYLOAD) { st = 0; break; }
        ln = b; sum = b; st = 3; break;
    case 3:
        cmd = b; sum += b; idx = 0;
        st = ln ? 4 : 5; break;
    case 4:
        buf[idx++] = b; sum += b;
        if (idx >= ln) st = 5;
        break;
    case 5:
        if ((uint8_t)(sum & 0xFF) == b)
            vision_on_frame(cmd, buf, ln);
        st = 0;
        break;
    default: st = 0; break;
    }
}

/* ------------ 发送(uart_send: 你工程里的串口发送函数指针) ------------ */
typedef void (*uart_send_fn)(const uint8_t *data, uint16_t len);

static void vision_send(uart_send_fn tx, uint8_t cmd, const uint8_t *pl, uint8_t ln)
{
    uint8_t frame[4 + V_MAX_PAYLOAD + 1];
    uint16_t sum = ln + cmd;
    frame[0] = V_HEAD0; frame[1] = V_HEAD1; frame[2] = ln; frame[3] = cmd;
    for (uint8_t i = 0; i < ln; i++) { frame[4 + i] = pl[i]; sum += pl[i]; }
    frame[4 + ln] = (uint8_t)(sum & 0xFF);
    tx(frame, 5 + ln);
}

void vision_send_set_mode(uart_send_fn tx, uint8_t mode) { vision_send(tx, VCMD_SET_MODE, &mode, 1); }
void vision_send_ping(uart_send_fn tx)                   { vision_send(tx, VCMD_PING, 0, 0); }
