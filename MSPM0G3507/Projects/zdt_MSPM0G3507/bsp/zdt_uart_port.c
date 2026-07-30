/**
 * @file zdt_uart_port.c
 * @brief MSPM0 UART port for ZDT Emm V5 stepper driver
 *
 * Uses UART_1 (UART3 peripheral, PA23 TX / PA24 RX) for motor communication.
 * Uses SysTick for millisecond tick.
 *
 * NOTE: The UART RX ISR (UART_1_INST_IRQHandler) is defined in
 * stepper_control.c because it needs access to the static g_motor instance.
 */
#include "zdt_uart_port.h"
#include "ti_msp_dl_config.h"

/* Millisecond tick counter (incremented by SysTick) */
static volatile uint32_t g_tick_ms = 0;

/* TX timeout per byte in milliseconds */
#define TX_BYTE_TIMEOUT_MS  2

/**
 * @brief SysTick interrupt handler — increments millisecond counter
 */
void SysTick_Handler(void)
{
    g_tick_ms++;
}

void ZDT_Port_Init(void)
{
    /* Enable UART RX interrupt for motor communication */
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    /* Configure RX pin pull-up (helps with idle line stability) */
    DL_GPIO_setDigitalInternalResistor(
        GPIO_UART_1_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);

    /* Start SysTick for millisecond timing */
    SysTick->LOAD = (CPUCLK_FREQ / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk    |
                    SysTick_CTRL_ENABLE_Msk;
}

bool ZDT_Port_Transmit(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint32_t deadline = g_tick_ms + TX_BYTE_TIMEOUT_MS;

        /* Wait for TX FIFO to have space (with timeout) */
        while (DL_UART_isTXFIFOFull(UART_1_INST)) {
            if ((int32_t)(deadline - g_tick_ms) <= 0) {
                return false;  /* TX timeout */
            }
        }

        DL_UART_Main_transmitData(UART_1_INST, data[i]);
    }

    return true;
}

uint32_t ZDT_Port_GetTickMs(void)
{
    return g_tick_ms;
}
