#include "show.h"
#include "oled.h"

extern int Motor1_Speed;
extern int Motor2_Speed;
extern int Motor1_T_Position;
extern int Motor2_T_Position;
extern int Motor1_Current_Position;
extern int Motor2_Current_Position;

static void OLED_ShowSignedValue(uint8_t x, uint8_t y, int value, uint8_t len)
{
    if (value < 0) {
        OLED_ShowChar(x, y, '-', 16);
        OLED_ShowNum(x + 8, y, (uint32_t)(-value), len, 16);
    } else {
        OLED_ShowChar(x, y, ' ', 16);
        OLED_ShowNum(x + 8, y, (uint32_t)value, len, 16);
    }
}

void OLED_Show(void)
{
    OLED_ShowString(0, 0, "F32C POS", 12);

    OLED_ShowString(0, 16, "T1:", 12);
    OLED_ShowSignedValue(18, 16, Motor1_T_Position, 6);
    OLED_ShowString(64, 16, "P1:", 12);
    OLED_ShowSignedValue(82, 16, Motor1_Current_Position, 6);

    OLED_ShowString(0, 32, "T2:", 12);
    OLED_ShowSignedValue(18, 32, Motor2_T_Position, 6);
    OLED_ShowString(64, 32, "P2:", 12);
    OLED_ShowSignedValue(82, 32, Motor2_Current_Position, 6);

    OLED_ShowString(0, 48, "SPD:", 12);
    OLED_ShowSignedValue(30, 48, Motor1_Speed, 4);
    OLED_ShowSignedValue(72, 48, Motor2_Speed, 4);

    OLED_Refresh();
}
