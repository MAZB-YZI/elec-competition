/*
 * Temporary CCS bridge: compile the reusable module implementation without
 * hand-editing .cproject. Convert this file to a CCS linked source entry later.
 */

/* 映射 OLED_INST 到 SysConfig 生成的 I2C_OLED_INST */
#define OLED_INST I2C_OLED_INST

#include "../../../../Modules/Drivers/OLED/oled.c"
