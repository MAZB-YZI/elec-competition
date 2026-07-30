/**
 * @file linkage_calibration.c
 * @brief Linkage angle-to-motor-position calibration
 *
 * Default table — update with real measured values after assembly.
 * Out-of-range inputs return false (no silent clamping).
 */
#include "linkage_calibration.h"

static LinkageCalPoint g_table[] = {
    /*  angle_mdeg,  motor_position */
    {     -3000,         -600       },
    {     -2000,         -350       },
    {     -1000,         -160       },
    {         0,            0       },
    {      1000,          155       },
    {      2000,          345       },
    {      3000,          590       },
};

#define TABLE_SIZE  (sizeof(g_table) / sizeof(g_table[0]))

void Linkage_Init(void)
{
    /* Static init — could load from flash in the future */
}

bool Linkage_AngleToPosition(int16_t angle_mdeg, int32_t *position)
{
    if (!position) return false;

    /* Out of range — reject */
    if (angle_mdeg < g_table[0].angle_mdeg ||
        angle_mdeg > g_table[TABLE_SIZE - 1].angle_mdeg) {
        return false;
    }

    /* Exact match at endpoints */
    if (angle_mdeg == g_table[0].angle_mdeg) {
        *position = g_table[0].motor_position;
        return true;
    }
    if (angle_mdeg == g_table[TABLE_SIZE - 1].angle_mdeg) {
        *position = g_table[TABLE_SIZE - 1].motor_position;
        return true;
    }

    /* Linear interpolation between adjacent points */
    for (uint8_t i = 0; i < TABLE_SIZE - 1; i++) {
        int16_t a0 = g_table[i].angle_mdeg;
        int16_t a1 = g_table[i + 1].angle_mdeg;

        if (angle_mdeg >= a0 && angle_mdeg <= a1) {
            int32_t p0 = g_table[i].motor_position;
            int32_t p1 = g_table[i + 1].motor_position;
            int32_t da = (int32_t)(angle_mdeg - a0);
            int32_t dp = p1 - p0;
            int32_t da_total = (int32_t)(a1 - a0);

            *position = p0 + (da * dp) / da_total;
            return true;
        }
    }

    return false; /* Should not reach here if table is sorted */
}

bool Linkage_PositionToAngle(int32_t position, int16_t *angle_mdeg)
{
    if (!angle_mdeg) return false;

    if (position < g_table[0].motor_position ||
        position > g_table[TABLE_SIZE - 1].motor_position) {
        return false;
    }

    if (position == g_table[0].motor_position) {
        *angle_mdeg = g_table[0].angle_mdeg;
        return true;
    }
    if (position == g_table[TABLE_SIZE - 1].motor_position) {
        *angle_mdeg = g_table[TABLE_SIZE - 1].angle_mdeg;
        return true;
    }

    for (uint8_t i = 0; i < TABLE_SIZE - 1; i++) {
        int32_t p0 = g_table[i].motor_position;
        int32_t p1 = g_table[i + 1].motor_position;

        if (position >= p0 && position <= p1) {
            int16_t a0 = g_table[i].angle_mdeg;
            int16_t a1 = g_table[i + 1].angle_mdeg;
            int32_t dp = position - p0;
            int32_t da = (int32_t)(a1 - a0);
            int32_t dp_total = p1 - p0;

            *angle_mdeg = a0 + (int16_t)((dp * da) / dp_total);
            return true;
        }
    }

    return false;
}

const LinkageCalPoint *Linkage_GetTable(uint8_t *count)
{
    if (count) *count = (uint8_t)TABLE_SIZE;
    return g_table;
}

int Linkage_SetPoint(uint8_t index, int16_t angle, int32_t position)
{
    if (index >= TABLE_SIZE) return -1;

    /* Validate sort order: angle must be strictly increasing */
    if (index > 0 && angle <= g_table[index - 1].angle_mdeg) return -1;
    if (index < TABLE_SIZE - 1 && angle >= g_table[index + 1].angle_mdeg) return -1;

    g_table[index].angle_mdeg    = angle;
    g_table[index].motor_position = position;
    return 0;
}
