/**
  ******************************************************************************
  * @file    app_a.c
  * @brief   A Board application — signal detection, LED control,
  *          communication, and display for the competition.
  ******************************************************************************
  */
 
#include "app_a.h"
#include "ssd1306.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <math.h>
 
/* ================================================================
 * Constants
 * ================================================================ */
 
/* TIM2 (D1 breathing PWM on PA5) — target 1 kHz */
#define TIM2_PSC            83
#define TIM2_ARR            999
 
/* TIM3 (1 kHz system timebase) */
#define TIM3_PSC            83
#define TIM3_ARR            999
 
/* ADC */
#define ADC_DMA_BUF_SIZE    8
#define SAMPLE_BUF_SIZE     2048    /* 2 s @ 1 kHz — ensures full 1-3Hz cycle */
#define VREF                3.30f
#define ADC_MAX             4095.0f
 
/* Signal detection thresholds (12-bit LSB) */
#define SIG_THRESH_LSB      80      /* ~64 mV pp → below = DC */
#define NOISE_STD_THRESH    25.0f   /* Std dev > this → noisy/floating */
#define NOINPUT_AVG_LSB     60      /* ~48 mV: avg below this + quiet → no input */
#define SQWAVE_RATIO        4.0f    /* max_deriv / avg_deriv → square */
 
/* Communication */
#define COMM_INTERVAL_TICK  50      /* Send packet every 50 ms (50 × 1 kHz) */
#define COMM_TIMEOUT_MS     800
 
/* Button K1 (PA2) */
#define LONG_PRESS_MS       800
#define DEBOUNCE_MS         30
 
/* Breathing frequencies for the 3 PWM modes */
#define BREATH_FREQ_MODE0   0.5f    /* 2 s cycle */
#define BREATH_FREQ_MODE1   1.0f    /* 1 s cycle */
#define BREATH_FREQ_MODE2   2.0f    /* 0.5 s cycle */

/* Breathing mode display labels */
#define BREATH_LABEL_SLOW   "Br:Slow"
#define BREATH_LABEL_MID    "Br:Mid"
#define BREATH_LABEL_FAST   "Br:Fast"
 
/* Display refresh period (ms) */
#define DISP_REFRESH_MS     300
 
/* Phase constant */
#define TWO_PI              6.283185307f
 
/* ================================================================
 * Global State
 * ================================================================ */
SystemState_t g_state;
 
/* Shared UART RX byte (accessed from stm32f4xx_it.c callback) */
volatile uint8_t uart_rx_byte;
 
/* ================================================================
 * Private Variables
 * ================================================================ */
 
/* ADC DMA circular buffer */
static uint16_t adc_dma_buf[ADC_DMA_BUF_SIZE];
 
/* Sample accumulation (filled from TIM3 ISR @ 1 kHz) */
static uint16_t sample_buf[SAMPLE_BUF_SIZE];
static volatile uint16_t sample_cnt = 0;
static float    adc_ema = 0.0f;        /* EMA-filtered ADC value */
static uint8_t  adc_ema_init = 0;      /* First-sample flag */
 
/* System tick @ 1 kHz (incremented in TIM3 ISR) */
static volatile uint32_t sys_tick = 0;
 
/* Communication */
static uint8_t tx_seq = 0;
static uint8_t comm_first_rx = 0;    /* set after first valid B packet */
static uint8_t last_b_seq = 0;       /* 记录收到的B板seq，回传给B板确认 */
 
/* Button state machine (runs entirely in TIM3 ISR @ 1kHz) */
typedef enum {
    BTN_IDLE = 0,
    BTN_DEBOUNCE_DOWN,
    BTN_PRESSED,
    BTN_DEBOUNCE_UP
} BtnState_t;
 
static BtnState_t btn_state = BTN_IDLE;
static uint32_t   btn_press_start = 0;   /* sys_tick when press confirmed */
static uint8_t    btn_debounce = 0;       /* consecutive stable samples */
static uint8_t    btn_long_sent = 0;      /* D2 toggle flagged for Comm_Send */
static uint8_t    btn_long_done = 0;      /* long-press fired this press cycle, prevent re-trigger */
 
/* Display periodic-refresh timestamp */
static uint32_t last_disp_ms = 0;
 
/* D2 command pending: set when K1 long-press fires during COMM_LOST,
   cleared and re-sent on the first successful re-connection packet. */
static uint8_t d2_cmd_pending = 0;
 
/* Forward declarations */
static void Signal_Analyze(void);
static void Comm_Send(void);
static void Button_Tick(void);
static void Display_Update(void);
static inline float ADCToV(uint16_t raw);
static int  FmtFloat(char *buf, float v, int decimals);
static int  FmtInt(char *buf, int val);
 
/* ================================================================
 * Helper: ADC raw → voltage
 * ================================================================ */
static inline float ADCToV(uint16_t raw)
{
    return (float)raw * VREF / ADC_MAX;
}
 
/* ================================================================
 * Helper: Format float into buf (handles 0.00 – 3.30 range, 2 dec.)
 * Returns string length.
 * ================================================================ */
static int FmtFloat(char *buf, float v, int decimals)
{
    if (v < 0.0f) { *buf++ = '-'; v = -v; }
    if (v > 99.99f) v = 99.99f;   /* clamp */
 
    int ip = (int)v;
    int fp;
    if (decimals == 2) {
        fp = (int)((v - (float)ip) * 100.0f + 0.5f);
        if (fp >= 100) { ip++; fp = 0; }
    } else {
        fp = (int)((v - (float)ip) * 10.0f + 0.5f);
        if (fp >= 10) { ip++; fp = 0; }
    }
 
    char *start = buf;
    if (ip >= 10) *buf++ = (char)('0' + ip / 10);
    *buf++ = (char)('0' + ip % 10);
    *buf++ = '.';
    if (decimals == 2) {
        *buf++ = (char)('0' + fp / 10);
        *buf++ = (char)('0' + fp % 10);
    } else {
        *buf++ = (char)('0' + fp);
    }
    *buf = '\0';
    return (int)(buf - start);
}
 
/* ================================================================
 * Helper: Format integer
 * ================================================================ */
static int FmtInt(char *buf, int val)
{
    char *start = buf;
    if (val < 0) { *buf++ = '-'; val = -val; }
    if (val >= 1000) *buf++ = (char)('0' + val / 1000);
    if (val >= 100)  *buf++ = (char)('0' + (val / 100) % 10);
    if (val >= 10)   *buf++ = (char)('0' + (val / 10) % 10);
    *buf++ = (char)('0' + val % 10);
    *buf = '\0';
    return (int)(buf - start);
}
 
/* ================================================================
 * Signal Analysis: classify & measure Vi
 * Called from main loop when enough samples are available.
 * ================================================================ */
static void Signal_Analyze(void)
{
    if (sample_cnt < 1000) return;          /* need >= 1 s for 1 Hz full cycle */
 
    /* Atomically grab snapshot */
    __disable_irq();
    uint16_t cnt = sample_cnt;
    sample_cnt = 0;
    __enable_irq();
 
    if (cnt < 1000) return;
 
    /* --- First pass: min / max / sum / variance ---------------- */
    uint16_t vmin = 4095, vmax = 0;
    uint32_t sum  = 0;
    float    sum_sq = 0.0f;
 
    for (uint16_t i = 0; i < cnt; i++) {
        uint16_t v = sample_buf[i];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        sum   += v;
        sum_sq += (float)v * (float)v;
    }
 
    float  avg_raw = (float)sum / (float)cnt;
    uint16_t pp    = vmax - vmin;
    float  variance = (sum_sq / (float)cnt) - (avg_raw * avg_raw);
    float  std_dev  = (variance > 0.0f) ? sqrtf(variance) : 0.0f;
 
    /* Store instantaneous value (use last filtered sample) */
    g_state.vi_instant = ADCToV(sample_buf[cnt - 1]);
 
    /* --- Classification ---------------------------------------- */
 
    /* Floating / noisy pin detection:
       A real DC source has low std_dev. A floating pin has
       higher std_dev even if peak-to-peak is moderate. */
    uint8_t noisy = (std_dev > NOISE_STD_THRESH) ? 1 : 0;
 
    /* No input: very low average AND (quiet or the pin is near 0V steady) */
    if (avg_raw < (float)NOINPUT_AVG_LSB && !noisy) {
        g_state.signal_type = SIGNAL_NONE;
        g_state.voltage     = 0.0f;
        return;
    }
 
    /* Floating pin detected — treat as "no input" regardless of avg */
    if (noisy && pp < (SIG_THRESH_LSB * 4)) {
        /* High noise but small swing → probably floating */
        g_state.signal_type = SIGNAL_NONE;
        g_state.voltage     = 0.0f;
        return;
    }
 
    /* DC: low peak-to-peak and not too noisy */
    if (pp < SIG_THRESH_LSB) {
        g_state.signal_type = SIGNAL_DC;
        g_state.voltage     = ADCToV((uint16_t)avg_raw);
        if (g_state.voltage < 0.01f) g_state.voltage = 0.0f;
        return;
    }
 
    /* AC signal — derivative-based sine/square discrimination */
    float sum_d = 0.0f, max_d = 0.0f;
    for (uint16_t i = 1; i < cnt; i++) {
        float d = (float)((int16_t)sample_buf[i] - (int16_t)sample_buf[i - 1]);
        if (d < 0.0f) d = -d;
        sum_d += d;
        if (d > max_d) max_d = d;
    }
    float avg_d = sum_d / (float)(cnt - 1);
 
    if (avg_d > 0.5f && (max_d / avg_d) > SQWAVE_RATIO) {
        g_state.signal_type = SIGNAL_SQUARE;
    } else {
        g_state.signal_type = SIGNAL_SINE;
    }
 
    /* Amplitude = Vpp / 2 */
    g_state.voltage = ADCToV((uint16_t)pp) / 2.0f;
}
 
/* ================================================================
 * Communication: send packet A→B (called every 50 ms from ISR)
 *
 * Packet (7 bytes):
 *   [0xAA] [seq] [flags] [vi_h] [vi_l] [last_b_seq] [xor-checksum]
 *   flags:       bit0 = toggle D2 request, bit1 = D1 status
 *   last_b_seq:  A板确认收到的B板seq（B板用这个判断A板是否收到自己的包）
 * ================================================================ */
static void Comm_Send(void)
{
    uint8_t  buf[7];
    uint16_t vi_mv = (uint16_t)(g_state.vi_instant * 1000.0f + 0.5f);
    uint8_t  flags = 0;
 
    if (btn_long_sent) {
        if (g_state.comm_state == COMM_LOST) {
            /* Link is down — save the command, send after reconnection */
            d2_cmd_pending = 1;
            btn_long_sent  = 0;
        } else {
            flags |= 0x01;        /* request B board to toggle D2 */
            btn_long_sent  = 0;
            d2_cmd_pending = 0;   /* normal send clears any pending too */
        }
    }

    /* D1 状态放入 bit1，让 B 板 OLED 显示 */
    if (g_state.d1_enabled) flags |= 0x02;
 
    buf[0] = 0xAA;
    buf[1] = tx_seq++;
    buf[2] = flags;
    buf[3] = (uint8_t)(vi_mv >> 8);
    buf[4] = (uint8_t)(vi_mv & 0xFF);
    buf[5] = last_b_seq;   /* 告诉B板：我最后收到你的seq是多少 */
    buf[6] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
 
    HAL_UART_Transmit(&huart1, buf, 7, 10);
}
 
/* ================================================================
 * Communication: process received byte from B→A
 *
 * B→A packet (5 bytes):
 *   [0xBB] [seq] [flags] [confirmed_seq] [xor-checksum]
 *   seq:           递增序列号（防 checksum 与同步字碰撞）
 *   flags:         bit0 = D2 state, bit1-2 = PWM mode
 *   confirmed_seq: B板确认收到的A板seq（用于A板判断B板是否收到自己的包）
 *
 * Called from HAL_UART_RxCpltCallback (interrupt context).
 * ================================================================ */
static uint8_t b_confirmed_seq = 0;  /* B板确认的A板seq */

void AppA_UART_RxCplt(uint8_t byte)
{
    static uint8_t  st  = 0;    /* 0=sync, 1=seq, 2=flags, 3=conf_seq, 4=chk */
    static uint8_t  seq = 0;
    static uint8_t  flg = 0;
    static uint8_t  csq = 0;

    if (byte == 0xBB) {
        st = 1;
        return;
    }

    if (st == 1) {
        seq = byte;
        st  = 2;
        return;
    }

    if (st == 2) {
        flg = byte;
        st  = 3;
        return;
    }

    if (st == 3) {
        csq = byte;
        st  = 4;
        return;
    }

    if (st == 4) {
        /* Verify: XOR of 0xBB, seq, flags, confirmed_seq must equal checksum */
        if ((uint8_t)(0xBB ^ seq ^ flg ^ csq) == byte) {
            comm_first_rx       = 1;
            b_confirmed_seq     = csq;
            last_b_seq          = seq;  /* 记录B板seq，下次发包带回 */
            g_state.d2_enabled  =  flg & 0x01;
            g_state.pwm_mode    = (PWMMode_t)((flg >> 1) & 0x03);
            g_state.last_rx_tick = HAL_GetTick();
            if (g_state.comm_state == COMM_LOST) {
                g_state.comm_state = COMM_OK;
                if (d2_cmd_pending) {
                    btn_long_sent  = 1;
                    d2_cmd_pending = 0;
                }
            }
        }
        st = 0;
    } else {
        st = 0;
    }
}
 
/* ================================================================
 * Communication: check timeout (called from main loop)
 * ================================================================ */
static void Comm_CheckTimeout(void)
{
    /* Startup grace period: don't flag LOST before first contact */
    if (!comm_first_rx && HAL_GetTick() < 3000) {
        return;
    }
    /* 超时判断：超过1秒没收到对方包 */
    if (HAL_GetTick() - g_state.last_rx_tick > COMM_TIMEOUT_MS) {
        g_state.comm_state = COMM_LOST;
    }
    /* seq差值判断：只有建立过通信后才检查，避免上电时误判 */
    if (comm_first_rx) {
        int8_t seq_diff = (int8_t)(tx_seq - b_confirmed_seq);
        if (seq_diff > 15) {
            g_state.comm_state = COMM_LOST;
        }
    }
}
 
/* ================================================================
 * Breathing LED (TIM2 CH1 on PA5) — duty-cycle update
 * Called from TIM3 ISR every 1 ms.
 *
 * On comm loss: keeps breathing at last-known frequency.
 * When D1 off: 0 % duty.
 * ================================================================ */
static void Breath_Update(void)
{
    if (!g_state.d1_enabled) {
        /* D1 off — zero duty, but keep phase running so resume is smooth */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        /* Still advance phase slowly so restart looks natural */
        g_state.breath_phase += 0.001f;
        if (g_state.breath_phase >= TWO_PI) g_state.breath_phase -= TWO_PI;
        return;
    }
 
    /* Select breathing frequency from PWM mode */
    float freq;
    switch (g_state.pwm_mode) {
        default:
        case PWM_MODE_FOLLOW: freq = BREATH_FREQ_MODE0; break;
        case PWM_MODE_MAX:    freq = BREATH_FREQ_MODE1; break;
        case PWM_MODE_MIN:    freq = BREATH_FREQ_MODE2; break;
    }
 
    /* Phase step per ms */
    float step = TWO_PI * freq / 1000.0f;
 
    /* Sinusoidal brightness 0.02 – 1.0 (ensure visible glimmer at min) */
    float bright = 0.49f * (1.0f + sinf(g_state.breath_phase)) + 0.02f;
    if (bright > 1.0f) bright = 1.0f;
 
    uint32_t pulse = (uint32_t)(bright * (float)(TIM2_ARR + 1));
    if (pulse > (uint32_t)TIM2_ARR) pulse = (uint32_t)TIM2_ARR;
 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
 
    /* Advance phase */
    g_state.breath_phase += step;
    if (g_state.breath_phase >= TWO_PI) g_state.breath_phase -= TWO_PI;
}
 
/* ================================================================
 * Button K1 processing (called from main loop)
 *
 * EXTI detects falling edge → we record timestamp.
 * Here we poll the pin level to detect release and classify
 * short / long press.
 * ================================================================ */
/* ================================================================
 * Button Tick — full debounce + press classifier
 * Runs in TIM3 ISR at 1 kHz.  No more main-loop dependency.
 * ================================================================ */
static void Button_Tick(void)
{
    uint8_t raw = (GPIOA->IDR & GPIO_PIN_2) ? 1 : 0;   /* 1 = high (not pressed) */
 
    switch (btn_state) {
 
    case BTN_IDLE:
        if (raw == 0) {                                 /* pin low → maybe pressing */
            if (btn_debounce == 0) {
                btn_press_start = sys_tick;             /* capture start time immediately */
            }
            if (++btn_debounce >= DEBOUNCE_MS) {        /* 30 consecutive lows */
                btn_state        = BTN_PRESSED;
                btn_long_sent    = 0;
                btn_long_done    = 0;
                btn_debounce     = 0;
                /* btn_press_start already set at first low */
            }
        } else {
            btn_debounce = 0;
        }
        break;
 
    case BTN_PRESSED:
    {
        uint32_t held = sys_tick - btn_press_start;     /* includes debounce time */
 
        /* Long-press action: fire once per press cycle */
        if (held >= LONG_PRESS_MS && !btn_long_done) {
            btn_long_sent = 1;          /* toggle D2 flag → sent by Comm_Send */
            btn_long_done = 1;          /* prevent re-trigger until release */
        }
 
        if (raw == 1) {                                 /* pin high → maybe releasing */
            if (++btn_debounce >= DEBOUNCE_MS) {        /* 30 consecutive highs */
                /* --- confirmed release --- */
                if (held < LONG_PRESS_MS) {
                    /* Short press → toggle D1 */
                    g_state.d1_enabled = !g_state.d1_enabled;
                    if (g_state.d1_enabled) {
                        g_state.breath_phase = 0.0f;
                    }
                }
                /* else: long press — D2 toggle already flagged above */
                btn_state    = BTN_IDLE;
                btn_debounce = 0;
            }
        } else {
            btn_debounce = 0;                           /* still held, reset release counter */
        }
        break;
    }
 
    default:
        btn_state    = BTN_IDLE;
        btn_debounce = 0;
        break;
    }
}
 
/* ================================================================
 * EXTI2 callback — minimal, the real work is in Button_Tick
 * ================================================================ */
void AppA_EXTI2_Callback(void)
{
    /* No-op.  The TIM3 ISR samples the pin at 1 kHz,
       which is fast enough for a mechanical button. */
}
 
/* ================================================================
 * 1 kHz System Tick (called from TIM3 ISR)
 * ================================================================ */
void AppA_Tick1kHz(void)
{
    sys_tick++;
 
    /* --- collect ADC sample (with EMA low-pass filter) --------- */
    {
        uint16_t raw = adc_dma_buf[0];
        if (!adc_ema_init) {
            adc_ema = (float)raw;
            adc_ema_init = 1;
        } else {
            /* EMA: 15% new + 85% old → ~20ms settling, strong noise rejection */
            adc_ema = adc_ema * 0.85f + (float)raw * 0.15f;
        }
        if (sample_cnt < SAMPLE_BUF_SIZE) {
            sample_buf[sample_cnt++] = (uint16_t)adc_ema;
        }
    }
 
    /* --- breathing LED update ---------------------------------- */
    Breath_Update();
 
    /* --- communication send ------------------------------------ */
    if ((sys_tick % COMM_INTERVAL_TICK) == 0) {
        Comm_Send();
    }
 
    /* --- button debounce + short/long detection ---------------- */
    Button_Tick();
}
 
/* ================================================================
 * Display Update (called from main loop)
 * Uses SSD1306 OLED via I2C1.
 * ================================================================ */
static void Display_Update(void)
{
    char line[22];   /* 21 chars + NUL, fits 128 px with 6×8 font */
 
    SSD1306_Clear();
 
    /* --- Line 0: title ------------------------------------------- */
    SSD1306_DrawString(0, 0, "== A Board ==");
 
    /* --- Line 1: signal type & voltage --------------------------- */
    memset(line, ' ', sizeof(line));
    switch (g_state.signal_type) {
        case SIGNAL_NONE:   memcpy(line, "Sig: None     ", 14); break;
        case SIGNAL_DC:     memcpy(line, "Sig: DC  ", 9);  break;
        case SIGNAL_SINE:   memcpy(line, "Sig: Sine ", 10); break;
        case SIGNAL_SQUARE: memcpy(line, "Sig: Sq   ", 10); break;
    }
    /* Append voltage value after the label */
    {
        char  vs[8];
        int   pos = 8;  /* after "Sig: XX " */
        if (g_state.signal_type != SIGNAL_NONE) {
            FmtFloat(vs, g_state.voltage, 2);
            vs[4] = 'V';
            vs[5] = '\0';
            for (int i = 0; vs[i]; i++) line[pos++] = vs[i];
        }
        line[21] = '\0';
    }
    SSD1306_DrawString(1, 0, line);
 
    /* --- Line 2: D1 status + breathing note --------------------- */
    memset(line, ' ', sizeof(line));
    if (g_state.d1_enabled) {
        memcpy(line, "D1:ON  ", 7);
    } else {
        memcpy(line, "D1:OFF ", 7);
    }
    /* Append breathing mode label */
    {
        const char *mode_str = "Mode:?";
        switch (g_state.pwm_mode) {
            case PWM_MODE_FOLLOW: mode_str = BREATH_LABEL_SLOW; break;
            case PWM_MODE_MAX:    mode_str = BREATH_LABEL_MID;  break;
            case PWM_MODE_MIN:    mode_str = BREATH_LABEL_FAST; break;
        }
        for (int i = 0; mode_str[i]; i++) line[8 + i] = mode_str[i];
    }
    line[21] = '\0';
    SSD1306_DrawString(2, 0, line);
 
    /* --- Line 3: D2 status (remote B board) --------------------- */
    memset(line, ' ', sizeof(line));
    memcpy(line, "D2:", 3);
    if (g_state.d2_enabled) {
        memcpy(line + 3, "ON ", 3);
    } else {
        memcpy(line + 3, "OFF", 3);
    }
    /* Append mode description */
    {
        const char *m = " Mode:Follow";
        if (g_state.pwm_mode == PWM_MODE_MAX) m = " Mode:Max   ";
        if (g_state.pwm_mode == PWM_MODE_MIN) m = " Mode:Min   ";
        for (int i = 0; m[i]; i++) line[7 + i] = m[i];
    }
    line[21] = '\0';
    SSD1306_DrawString(3, 0, line);
 
    /* --- Line 4: Communication status --------------------------- */
    memset(line, ' ', sizeof(line));
    if (g_state.comm_state == COMM_OK) {
        memcpy(line, "Comm: OK  ", 10);
    } else {
        memcpy(line, "Comm: LOST", 10);
    }
    line[21] = '\0';
    SSD1306_DrawString(4, 0, line);
 
    /* --- Line 5: Raw ADC debug ----------------------------------- */
    memset(line, ' ', sizeof(line));
    {
        char tmp[8];
        memcpy(line, "ADC:", 4);
        int pos = 4;
        FmtInt(tmp, (int)adc_ema);
        for (int i = 0; tmp[i]; i++) line[pos++] = tmp[i];
        line[pos++] = ' ';
        line[pos++] = '=';
        line[pos++] = ' ';
        FmtFloat(tmp, ADCToV((uint16_t)adc_ema), 2);
        for (int i = 0; tmp[i]; i++) line[pos++] = tmp[i];
    }
    line[21] = '\0';
    SSD1306_DrawString(5, 0, line);
 
    /* --- Line 6: filtered sample buffer info --------------------- */
    {
        char num[5];
        FmtInt(num, (int)sample_cnt);
        SSD1306_DrawString(6, 0, "Buf:");
        SSD1306_DrawString(6, 4, num);
    }
 
    SSD1306_UpdateScreen();
}
 
/* ================================================================
 * AppA_Init: one-time initialization (call after all MX_*_Init)
 * ================================================================ */
void AppA_Init(void)
{
    /* Zero out state ---------------------------------------------- */
    memset(&g_state, 0, sizeof(g_state));
    g_state.vi_instant   = 1.65f;      /* 初始Vi=1.65V，防止发0给B板导致PWM=10% */
    g_state.comm_state   = COMM_OK;
    g_state.pwm_mode     = PWM_MODE_FOLLOW;
    g_state.last_rx_tick = HAL_GetTick();
 
    /* --- Start TIM2 PWM (1 kHz on PA5, CubeMX已配置PSC=83/ARR=999) -- */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 
    /* --- Start TIM3 timebase (1 kHz, CubeMX已配置PSC=83/ARR=999) --- */
    HAL_TIM_Base_Start_IT(&htim3);
 
    /* --- Start ADC with DMA circular ---------------------------- */
    HAL_Delay(1);   /* let ADC stabilize after init */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_BUF_SIZE);
 
    /* --- Start UART RX (byte-by-byte interrupt) ----------------- */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_byte, 1);
 
    /* --- Init SSD1306 & show boot screen ------------------------ */
    /* (SSD1306_Init was already called before AppA_Init in main.c,
       but if not, uncomment below) */
    /* SSD1306_Init(); */
 
    SSD1306_Clear();
    SSD1306_DrawString(2, 0, "   A Board Boot   ");
    SSD1306_DrawString(4, 0, "   Initializing   ");
    SSD1306_UpdateScreen();
    HAL_Delay(500);
}
 
/* ================================================================
 * AppA_Process: main-loop tasks (non-ISR context)
 * ================================================================ */
void AppA_Process(void)
{
    uint32_t now = HAL_GetTick();
 
    /* --- Signal analysis (triggered when >= 1 s of data ready) -- */
    if (sample_cnt >= 1000) {
        Signal_Analyze();
    }
 
    /* --- Communication timeout check ---------------------------- */
    Comm_CheckTimeout();
 
    /* --- Display refresh (throttled) ---------------------------- */
    if (now - last_disp_ms >= DISP_REFRESH_MS) {
        last_disp_ms = now;
        Display_Update();
    }
 
    /* --- Breathing phase slowly drifts even when off, so the
       restart is smooth. Nothing extra needed here. -------------- */
}