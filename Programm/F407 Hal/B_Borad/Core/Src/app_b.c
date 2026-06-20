/**
  ******************************************************************************
  * @file    app_b.c
  * @brief   B Board application — complementary PWM, D2 LED,
  *          communication, and display for the competition.
  ******************************************************************************
  */

#include "app_b.h"
#include "ssd1306.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <math.h>

/* ================================================================
 * Constants
 * ================================================================ */

/* TIM1 complementary PWM (5 kHz, 168 MHz APB2 timer) */
#define TIM1_ARR            33599
#define TIM1_DUTY_MIN       0.10f
#define TIM1_DUTY_MAX       0.90f
#define TIM1_DUTY_INIT      0.50f

/* TIM2 D2 LED PWM (1 kHz, 84 MHz APB1 timer) */
#define TIM2_ARR            999
#define TIM2_DUTY_MIN       0.02f   /* micro-glow at Vi=0 */
#define TIM2_DUTY_MAX       0.90f

/* Communication */
#define COMM_INTERVAL_TICK  50      /* Send packet every 50 ms */
#define COMM_TIMEOUT_MS     800

/* K2 button debounce */
#define DEBOUNCE_MS         30

/* Vi history for max/min tracking (1s = 20 samples at 50ms rate) */
#define VI_HISTORY_SIZE     20

/* Display refresh */
#define DISP_REFRESH_MS     300

/* ================================================================
 * Global State
 * ================================================================ */
SystemState_t g_state;

/* Shared UART RX byte */
volatile uint8_t uart_rx_byte;

/* ================================================================
 * Private Variables
 * ================================================================ */

/* Vi history ring buffer */
static float   vi_history[VI_HISTORY_SIZE];
static uint8_t vi_hist_idx = 0;
static uint8_t vi_hist_full = 0;

/* System tick */
static volatile uint32_t sys_tick = 0;

/* Communication */
static uint8_t tx_seq = 0;
static uint8_t comm_first_rx = 0;
static uint8_t last_a_seq = 0;   /* 记录收到的A板seq，回传给A板确认 */
static uint8_t a_confirmed_seq = 0;  /* A板确认的B板seq */

/* K2 button */
static uint32_t k2_last_press_tick = 0;

/* Display */
static uint32_t last_disp_ms = 0;

/* ================================================================
 * Helpers
 * ================================================================ */
static int FmtFloat(char *buf, float v, int decimals)
{
    if (v < 0.0f) { *buf++ = '-'; v = -v; }
    if (v > 99.99f) v = 99.99f;
    int ip = (int)v;
    int fp = (int)((v - (float)ip) * 100.0f + 0.5f);
    if (fp >= 100) { ip++; fp = 0; }
    char *start = buf;
    if (ip >= 10) *buf++ = (char)('0' + ip / 10);
    *buf++ = (char)('0' + ip % 10);
    *buf++ = '.';
    *buf++ = (char)('0' + fp / 10);
    *buf++ = (char)('0' + fp % 10);
    *buf = '\0';
    return (int)(buf - start);
}

/* ================================================================
 * Map Vi (0–3.3V) to duty (dmin–dmax)
 * ================================================================ */
static float ViToDuty(float vi, float dmin, float dmax)
{
    if (vi <= 0.0f)  return dmin;
    if (vi >= 3.30f) return dmax;
    return dmin + (vi / 3.30f) * (dmax - dmin);
}

/* ================================================================
 * Add Vi sample to history ring buffer
 * ================================================================ */
static void ViHistory_Add(float vi)
{
    vi_history[vi_hist_idx] = vi;
    vi_hist_idx++;
    if (vi_hist_idx >= VI_HISTORY_SIZE) {
        vi_hist_idx = 0;
        vi_hist_full = 1;
    }
}

/* ================================================================
 * Get max Vi in history
 * ================================================================ */
static float ViHistory_GetMax(void)
{
    uint8_t n = vi_hist_full ? VI_HISTORY_SIZE : vi_hist_idx;
    if (n == 0) return 0.0f;
    float m = vi_history[0];
    for (uint8_t i = 1; i < n; i++) {
        if (vi_history[i] > m) m = vi_history[i];
    }
    return m;
}

/* ================================================================
 * Get min Vi in history
 * ================================================================ */
static float ViHistory_GetMin(void)
{
    uint8_t n = vi_hist_full ? VI_HISTORY_SIZE : vi_hist_idx;
    if (n == 0) return 0.0f;
    float m = vi_history[0];
    for (uint8_t i = 1; i < n; i++) {
        if (vi_history[i] < m) m = vi_history[i];
    }
    return m;
}

/* ================================================================
 * Update complementary PWM duty on TIM1 CH1/CH1N
 * ================================================================ */
static void UpdatePWM(void)
{
    float duty;

    if (g_state.comm_state == COMM_LOST) {
        /* Keep last duty — g_state.pwm_duty unchanged */
        duty = g_state.pwm_duty;
    } else {
        switch (g_state.pwm_mode) {
            default:
            case PWM_MODE_FOLLOW:
                duty = ViToDuty(g_state.vi_voltage, TIM1_DUTY_MIN, TIM1_DUTY_MAX);
                break;
            case PWM_MODE_MAX:
                duty = ViToDuty(ViHistory_GetMax(), TIM1_DUTY_MIN, TIM1_DUTY_MAX);
                break;
            case PWM_MODE_MIN:
                duty = ViToDuty(ViHistory_GetMin(), TIM1_DUTY_MIN, TIM1_DUTY_MAX);
                break;
        }
        g_state.pwm_duty = duty;
    }

    uint32_t pulse = (uint32_t)(duty * (float)(TIM1_ARR + 1));
    if (pulse > (uint32_t)TIM1_ARR) pulse = (uint32_t)TIM1_ARR;

    /* Update both CH1 and CH1N (complementary — same CCR, HW inverts CH1N) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

/* ================================================================
 * Update D2 LED brightness on TIM2 CH1
 * ================================================================ */
static void UpdateD2(void)
{
    if (!g_state.d2_enabled) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        return;
    }

    float duty;
    if (g_state.comm_state == COMM_LOST) {
        /* Keep last brightness */
        duty = g_state.d2_duty;
    } else {
        duty = ViToDuty(g_state.vi_voltage, TIM2_DUTY_MIN, TIM2_DUTY_MAX);
        g_state.d2_duty = duty;
    }

    uint32_t pulse = (uint32_t)(duty * (float)(TIM2_ARR + 1));
    if (pulse > (uint32_t)TIM2_ARR) pulse = (uint32_t)TIM2_ARR;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

/* ================================================================
 * Communication: send packet B→A
 * Packet: [0xBB] [seq] [flags] [checksum]
 *   seq:   每次发送递增，防止 FOLLOW+D2off 时 flags=0x00 导致
 *          checksum 固定等于 0xBB（与同步字碰撞）
 *   flags: bit0 = D2 state, bit1-2 = PWM mode
 * ================================================================ */
static void Comm_Send(void)
{
    uint8_t buf[5];
    uint8_t flags = 0;

    if (g_state.d2_enabled)  flags |= 0x01;
    flags |= (uint8_t)((g_state.pwm_mode & 0x03) << 1);

    buf[0] = 0xBB;
    buf[1] = tx_seq++;
    buf[2] = flags;
    buf[3] = last_a_seq;   /* 告诉A板：我最后收到你的seq是多少 */
    buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

    HAL_UART_Transmit(&huart1, buf, 5, 10);
}

/* ================================================================
 * Communication: process received byte from A→B
 * Packet: [0xAA] [seq] [flags] [vi_h] [vi_l] [checksum]
 *   flags: bit0 = toggle D2 request
 * ================================================================ */
void AppB_UART_RxCplt(uint8_t byte)
{
    static uint8_t  st  = 0;
    static uint8_t  buf[6];
    static uint8_t  pos = 0;

    if (byte == 0xAA) {
        st  = 1;
        pos = 0;
        return;
    }

    if (st == 1) {
        buf[pos++] = byte;
        if (pos >= 6) {
            /* Verify checksum: XOR of all bytes except checksum */
            uint8_t chk = 0xAA;
            for (uint8_t i = 0; i < 5; i++) chk ^= buf[i];
            if (chk == buf[5]) {
                /* seq=buf[0], flags=buf[1], vi=(buf[2]<<8)|buf[3], last_b_seq=buf[4] */
                uint8_t flags   = buf[1];
                uint16_t vi_mv  = ((uint16_t)buf[2] << 8) | buf[3];
                float    vi_new = (float)vi_mv / 1000.0f;

                comm_first_rx     = 1;
                last_a_seq        = buf[0];  /* 记录A板seq，下次发包带回 */
                a_confirmed_seq   = buf[4];  /* A板确认的B板seq */

                /* Toggle D2 on K1 long press from A board */
                if (flags & 0x01) {
                    g_state.d2_enabled = !g_state.d2_enabled;
                }

                /* D1 状态从 bit1 读取 */
                g_state.d1_enabled = (flags >> 1) & 0x01;

                /* Update Vi and add to history */
                g_state.vi_voltage  = vi_new;
                ViHistory_Add(vi_new);

                /* Mark communication alive */
                g_state.last_rx_tick = HAL_GetTick();
                if (g_state.comm_state == COMM_LOST) {
                    g_state.comm_state = COMM_OK;
                }
            }
            st = 0;
        }
    }
}

/* ================================================================
 * Communication: check timeout
 * ================================================================ */
static void Comm_CheckTimeout(void)
{
    if (!comm_first_rx && HAL_GetTick() < 3000) {
        return;
    }
    /* 超时判断：超过1秒没收到对方包 */
    if (HAL_GetTick() - g_state.last_rx_tick > COMM_TIMEOUT_MS) {
        g_state.comm_state = COMM_LOST;
    }
    /* seq差值判断：只有建立过通信后才检查，避免上电时误判 */
    if (comm_first_rx) {
        int8_t seq_diff = (int8_t)(tx_seq - a_confirmed_seq);
        if (seq_diff > 15) {
            g_state.comm_state = COMM_LOST;
        }
    }
}

/* ================================================================
 * 1 kHz System Tick (called from TIM3 ISR)
 * ================================================================ */
void AppB_Tick1kHz(void)
{
    sys_tick++;

    /* --- Update D2 brightness (every tick for smooth response) -- */
    UpdateD2();

    /* --- Update complementary PWM duty (every tick) ------------- */
    UpdatePWM();

    /* --- Send status to A board (every 50 ms) ------------------- */
    if ((sys_tick % COMM_INTERVAL_TICK) == 0) {
        Comm_Send();
    }
}

/* ================================================================
 * EXTI2 callback: K2 press → cycle PWM mode
 * ================================================================ */
void AppB_EXTI2_Callback(void)
{
    uint32_t now = HAL_GetTick();

    /* Simple debounce: ignore presses within 200ms */
    if (now - k2_last_press_tick < 200) return;
    k2_last_press_tick = now;

    /* Cycle PWM mode */
    g_state.pwm_mode = (PWMMode_t)(((uint8_t)g_state.pwm_mode + 1) % 3);

    /* Clear history when mode changes (fresh start) */
    vi_hist_idx = 0;
    vi_hist_full = 0;
}

/* ================================================================
 * Display Update (called from main loop)
 * ================================================================ */
static void Display_Update(void)
{
    char line[22];

    SSD1306_Clear();

    /* Line 0: Title */
    SSD1306_DrawString(0, 0, "== B Board ==");

    /* Line 1: Vi voltage */
    memset(line, ' ', sizeof(line));
    memcpy(line, "Vi:", 3);
    {
        char vs[8];
        int pos = 3;
        FmtFloat(vs, g_state.vi_voltage, 2);
        for (int i = 0; vs[i]; i++) line[pos++] = vs[i];
        line[pos++] = 'V';
    }
    line[21] = '\0';
    SSD1306_DrawString(1, 0, line);

    /* Line 2: D1 status (A board remote) + PWM mode */
    memset(line, ' ', sizeof(line));
    if (g_state.d1_enabled) {
        memcpy(line, "D1:ON  ", 7);
    } else {
        memcpy(line, "D1:OFF ", 7);
    }
    {
        const char *m;
        switch (g_state.pwm_mode) {
            default:
            case PWM_MODE_FOLLOW: m = "Follow"; break;
            case PWM_MODE_MAX:    m = "Max   "; break;
            case PWM_MODE_MIN:    m = "Min   "; break;
        }
        for (int i = 0; m[i]; i++) line[7 + i] = m[i];
    }
    line[21] = '\0';
    SSD1306_DrawString(2, 0, line);

    /* Line 3: D2 status + PWM duty */
    memset(line, ' ', sizeof(line));
    if (g_state.d2_enabled) {
        memcpy(line, "D2:ON  ", 7);
    } else {
        memcpy(line, "D2:OFF ", 7);
    }
    {
        int  pct = (int)(g_state.pwm_duty * 100.0f + 0.5f);
        line[7] = 'P'; line[8] = ':';
        if (pct >= 10) { line[9] = (char)('0' + pct / 10); line[10] = (char)('0' + pct % 10); line[11] = '%'; }
        else           { line[9] = (char)('0' + pct);       line[10] = '%'; }
    }
    line[21] = '\0';
    SSD1306_DrawString(3, 0, line);

    /* Line 4: Communication status */
    memset(line, ' ', sizeof(line));
    if (g_state.comm_state == COMM_OK) {
        memcpy(line, "Comm: OK  ", 10);
    } else {
        memcpy(line, "Comm: LOST", 10);
    }
    line[21] = '\0';
    SSD1306_DrawString(4, 0, line);

    /* Line 5: PWM mode number (debug) */
    {
        char mline[22];
        memset(mline, ' ', sizeof(mline));
        memcpy(mline, "ModeNum:", 8);
        mline[8] = (char)('0' + (uint8_t)g_state.pwm_mode);
        mline[9] = ' ';
        /* Also show raw duty % */
        int pct2 = (int)(g_state.pwm_duty * 100.0f + 0.5f);
        memcpy(mline + 10, "Duty:", 5);
        if (pct2 >= 100) { mline[15]='1'; mline[16]=(char)('0'+(pct2/10)%10); mline[17]=(char)('0'+pct2%10); }
        else if (pct2 >= 10) { mline[15]=(char)('0'+pct2/10); mline[16]=(char)('0'+pct2%10); }
        else { mline[15]=(char)('0'+pct2); }
        mline[21] = '\0';
        SSD1306_DrawString(5, 0, mline);
    }
    /* Line 6: K2 hint */
    SSD1306_DrawString(6, 0, "K2:Follow/Max/Min ");

    SSD1306_UpdateScreen();
}

/* ================================================================
 * AppB_Init
 * ================================================================ */
void AppB_Init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.comm_state   = COMM_OK;
    g_state.pwm_mode     = PWM_MODE_FOLLOW;
    g_state.pwm_duty     = TIM1_DUTY_INIT;
    g_state.vi_voltage   = 1.65f;      /* 初始Vi=1.65V，跟随模式算出50%占空比 */
    g_state.last_rx_tick = HAL_GetTick();

    /* --- Start TIM1 complementary PWM (CH1 + CH1N) at 50% ------ */
    {
        uint32_t pulse = (uint32_t)(TIM1_DUTY_INIT * (float)(TIM1_ARR + 1));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);      /* CH1  + MOE */
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);   /* CH1N */
    }

    /* --- Start TIM2 PWM for D2 LED (off initially) ------------- */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /* --- Start TIM3 timebase (1 kHz) --------------------------- */
    HAL_TIM_Base_Start_IT(&htim3);

    /* --- Start UART RX (byte-by-byte interrupt) ----------------- */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_byte, 1);

    /* --- Boot screen -------------------------------------------- */
    SSD1306_Clear();
    SSD1306_DrawString(2, 0, "   B Board Boot   ");
    SSD1306_DrawString(4, 0, "   Initializing   ");
    SSD1306_UpdateScreen();
    HAL_Delay(500);
}

/* ================================================================
 * AppB_Process (main loop)
 * ================================================================ */
void AppB_Process(void)
{
    uint32_t now = HAL_GetTick();

    /* Communication timeout check */
    Comm_CheckTimeout();

    /* Display refresh */
    if (now - last_disp_ms >= DISP_REFRESH_MS) {
        last_disp_ms = now;
        Display_Update();
    }
}