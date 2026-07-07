# STM32F103C8T6 MaixCAM2 Vision UART Receiver

This STM32 HAL example now receives the current MaixCAM2 `binary_v1` vision packet.

## Wiring

Use USART2 on STM32F103C8T6:

```text
MaixCAM2 A21 / UART4_TX  ->  STM32 PA3 / USART2_RX
MaixCAM2 GND             ->  STM32 GND
```

Optional debug TX from STM32 is already on PA2, typically connected to a USB-TTL
adapter for `printf` output.

UART settings:

```text
115200 baud, 8 data bits, no parity, 1 stop bit
```

## Packet Format

MaixCAM2 sends fixed 19-byte packets:

```text
A5 5A 01 seq_lo seq_hi flags cx_lo cx_hi cy_lo cy_hi dx_lo dx_hi dy_lo dy_hi confidence fps bcc 0D 0A
```

`bcc` is XOR of bytes 2..15, from `type` through `fps`.

Payload decoded by STM32:

```c
seq         uint16_t
flags       uint8_t   bit0 found, bit1 stable
cx          int16_t
cy          int16_t
dx          int16_t   cx - setpoint_x
dy          int16_t   cy - setpoint_y
confidence  uint8_t   0..100
fps         uint8_t
```

## Modified Files

- `User/middleware/serial_protocol.c`
  - Changed from old variable-length `AA ... 55` protocol to MaixCAM2 fixed
    `A5 5A ... 0D 0A` protocol.
  - Public function names are unchanged: `packet_is_valid`, `packet_length`,
    `packet_encode`, `packet_decode`.

- `User/protocol_test.c`
  - Keeps USART2 interrupt receive and ringbuffer.
  - Decodes target fields into `g_vision_target`.
  - Prints:

```text
seq found stable cx cy dx dy conf fps count
```

## Bring-up

1. On MaixCAM2, set `RUN_MODE = "vision"` and `PROTOCOL_MODE = "binary_v1"`.
2. Connect MaixCAM2 A21 to STM32 PA3 and common GND.
3. Flash this STM32 project.
4. Open the STM32 debug USART/printf terminal.
5. Confirm `count` increases continuously.
6. Move the target and confirm `dx/dy` change.
7. If `count` does not increase, first check baud rate, GND, and whether MaixCAM2
   is running `vision` instead of `test_detector`.
