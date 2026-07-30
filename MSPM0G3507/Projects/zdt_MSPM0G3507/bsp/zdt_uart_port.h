/**
 * @file zdt_uart_port.h
 * @brief MSPM0 UART port layer for ZDT Emm V5 stepper driver
 *
 * Provides:
 * - UART TX with timeout (non-blocking FIFO check)
 * - UART RX ISR that feeds bytes into ZDT_RxByte()
 * - Millisecond tick from SysTick
 */
#ifndef ZDT_UART_PORT_H
#define ZDT_UART_PORT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize UART port (enable RX interrupt, etc.)
 *
 * Call after SYSCFG_DL_init().
 */
void ZDT_Port_Init(void);

/**
 * @brief Transmit buffer over motor UART (blocking with timeout)
 * @param data  Pointer to data bytes
 * @param len   Number of bytes to send
 * @return true if all bytes sent, false on timeout
 */
bool ZDT_Port_Transmit(const uint8_t *data, uint16_t len);

/**
 * @brief Get system tick in milliseconds
 * @return Millisecond counter value
 */
uint32_t ZDT_Port_GetTickMs(void);

/**
 * @brief Get motor UART instance (for advanced use)
 * @return Pointer to UART peripheral
 */
void *ZDT_Port_GetUart(void);

#endif /* ZDT_UART_PORT_H */
