#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

/* ── 帧格式（A/B 板共用同一结构，src 字段区分来源）── */
#define FRAME_HEAD   0xAA
#define FRAME_TAIL   0x55
#define FRAME_LEN    8       /* 总字节数 */

/* src 字段值 */
#define SRC_A        0x01
#define SRC_B        0x02

/* pwm_mode 字段值 */
#define MODE_FOLLOW  0       /* 跟随 Vi */
#define MODE_MAX     1       /* 过去 1s 最大值 */
#define MODE_MIN     2       /* 过去 1s 最小值 */

/* d2_cmd 字段值（A→B）*/
#define D2_CMD_NONE  0
#define D2_CMD_TOGGLE 1

/*
 *  字节布局
 *  [0] head      0xAA
 *  [1] src       0x01/0x02
 *  [2] d1_on     0/1  (发送方自己的 LED 状态)
 *  [3] d2_on     0/1
 *  [4] pwm_mode  0/1/2
 *  [5] d2_cmd    0/1  (仅 A→B 有效，B→A 填 0)
 *  [6] checksum  [0]^[1]^[2]^[3]^[4]^[5]
 *  [7] tail      0x55
 */
typedef struct {
    uint8_t head;
    uint8_t src;
    uint8_t d1_on;
    uint8_t d2_on;
    uint8_t pwm_mode;
    uint8_t d2_cmd;
    uint8_t checksum;
    uint8_t tail;
} CommFrame_t;

/* 帧编解码 */
static inline void Frame_Pack(CommFrame_t *f, uint8_t src,
                               uint8_t d1, uint8_t d2,
                               uint8_t mode, uint8_t d2cmd)
{
    f->head     = FRAME_HEAD;
    f->src      = src;
    f->d1_on    = d1;
    f->d2_on    = d2;
    f->pwm_mode = mode;
    f->d2_cmd   = d2cmd;
    f->checksum = f->head ^ f->src ^ f->d1_on ^
                  f->d2_on ^ f->pwm_mode ^ f->d2_cmd;
    f->tail     = FRAME_TAIL;
}

static inline int Frame_Verify(const uint8_t *buf)
{
    if (buf[0] != FRAME_HEAD) return 0;
    if (buf[7] != FRAME_TAIL) return 0;
    uint8_t cs = buf[0]^buf[1]^buf[2]^buf[3]^buf[4]^buf[5];
    return (cs == buf[6]);
}

#endif /* __PROTOCOL_H__ */
