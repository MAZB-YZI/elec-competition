# F32C_GIMBAL_MSPM0G3507

MSPM0G3507 control example for WHEELTEC F32C TTL brushless gimbal motors.

## Hardware wiring

Use the 12 V powered serial connector on the expansion board.

- MCU UART3 TX: PB2 -> F32C RX
- MCU UART3 RX: PB3 <- F32C TX
- 12V -> F32C VIN / 12V
- GND -> F32C GND, common ground with MSPM0G3507

The SysConfig instance is still named `UART_1` for compatibility with the original example code, but it is assigned to the MCU `UART3` peripheral.

## Default behavior

- Baud rate: 115200 8N1
- Motor IDs: 1 and 2
- Default mode: multi-turn position closed loop
- Button `PA18` increments both target positions by `900` protocol units, which the vendor example comments as 90.0 degrees.
- OLED pins follow the vendor example: RST PB14, DC PB15, SCL PA28, SDA PA31.

## Protocol notes

Frames use header `0x7A`, tail `0x7B`, and BCC XOR checksum over the bytes before the checksum byte.

Useful commands implemented in `empty.c`:

- enable motor: command `0x06`
- set mode: command `0x00`, value `1` for position closed loop
- set position speed: command `0x01`
- set target position: command `0x02`
- request position feedback: command `0x0E 0x01`

If you want velocity closed-loop instead, the original velocity example is in the vendor resource zip `WHEELTEC_C07A_F32C_Vel.zip`.
