#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "../../../../Modules/Drivers/JY61P/jy61p.h"

/*
 * JY61P 启动即显示状态版
 * 目标：
 *   1) 不再停在 Waiting...
 *   2) 启动后持续显示 RX / F53 / YawRaw / YawDeg
 *   3) 方便现场直接判断卡在收包还是解帧
 */

static void show_u32(uint8_t x, uint8_t y, const char *label, uint32_t value, uint8_t digits)
{
    OLED_ShowString(x, y, label, 12);
    OLED_ShowNum(x + 40, y, value, digits, 12);
}

static void show_signed_float(uint8_t x, uint8_t y, float value, uint8_t digits)
{
    OLED_ShowString(x, y, (value < 0.0f) ? "-" : "+", 12);
    OLED_ShowNum(x + 8, y, (uint32_t)(value < 0.0f ? -value : value), digits, 12);
}

int main(void)
{
    JY61P_Status status;
    uint32_t refresh_div = 0U;
    uint32_t sync_wait_ms = 0U;
    bool yaw_zeroed = false;

    SYSCFG_DL_init();
    OLED_Init();
    JY61P_Init();

    /*
     * 启动同步完成后，等到收到第一帧有效角度数据，再把当前姿态
     * 作为零点。这样可以保留“插着模块也能启动”的同步流程，
     * 同时恢复“上电自动归零开始”的使用习惯。
     */
    while (sync_wait_ms < 1500U) {
        (void)JY61P_UpdateStatus(&status);
        if (status.frame_count > 0U) {
            JY61P_ResetYaw();
            yaw_zeroed = true;
            break;
        }
        delay_ms(10U);
        sync_wait_ms += 10U;
    }

    if (yaw_zeroed == false) {
        /*
         * 超时也不要卡死：继续运行，但保持当前偏移。
         * 这样至少能继续观察 RX / F53 / YawRaw 是否恢复。
         */
        JY61P_ResetYaw();
    }

    while (1) {
        (void)JY61P_UpdateStatus(&status);

        if (++refresh_div >= 10U) {
            refresh_div = 0U;

            OLED_Clear();
            OLED_ShowString(0, 0, "JY61P Inspect", 12);

            show_u32(0, 16, "RX", status.rx_byte_count, 6);
            show_u32(0, 28, "F53", status.frame_count, 6);

            OLED_ShowString(0, 40, "YawRaw:", 12);
            OLED_ShowNum(56, 40, status.yaw_raw, 6, 12);

            OLED_ShowString(0, 52, "YawDeg:", 12);
            show_signed_float(56, 52, status.yaw_deg, 4);

            OLED_ShowString(96, 16, (status.frame_count > 0U) ? "OK" : "NO", 12);

            OLED_Refresh();
        }

        delay_ms(10U);
    }
}
