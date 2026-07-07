# F32C Gimbal Integration Notes

Current UART allocation in this project:

```text
USART1 PA9/PA10   debug printf terminal
USART2 PA2/PA3    MaixCAM2 binary_v1 receiver
USART3 PB10/PB11  WHEELTEC F32C TTL gimbal bus
```

## Wiring

MaixCAM2 receiver:

```text
MaixCAM2 A21 / UART4_TX -> STM32 PA3 / USART2_RX
MaixCAM2 GND            -> STM32 GND
```

F32C gimbal TTL bus:

```text
STM32 PB10 / USART3_TX -> F32C RXD
STM32 PB11 / USART3_RX -> F32C TXD, optional in this first version
STM32 GND              -> F32C GND
```

Do not connect F32C to USART2 in this merged firmware. USART2 is reserved for
MaixCAM2.

## Control Flow

`main.c` now runs:

```c
DEBUG_USART_Config();
protocol_test_init();
F32C_Gimbal_Init();

while (1) {
    protocol_test_proc();
    F32C_Gimbal_Task();
}
```

`F32C_Gimbal_Init()` does:

1. initialize USART3 at 115200;
2. send one `0x00` wake byte;
3. wait 1500 ms for F32C boot;
4. enable motor ID 1 and 2;
5. set both motors to multi-turn position mode with T trajectory planning;
6. set both motor speeds to 50;
7. send initial position 0.

`F32C_Gimbal_Task()` runs every 20 ms. When MaixCAM2 data is fresh and target is
found, it updates target position from `dx/dy` and sends both motor position
frames.

## Tuning Macros

Edit `User/f32c_gimbal.h`:

```c
#define F32C_YAW_DIR                1
#define F32C_PITCH_DIR              1
```

If one axis moves away from the target, change that axis direction to `-1`.

Conservative default step limit:

```c
#define F32C_YAW_STEP_LIMIT_X10     5
#define F32C_PITCH_STEP_LIMIT_X10   5
```

Position unit is 0.1 degree, so 5 means 0.5 degrees per 20 ms update.

Default gain:

```c
#define F32C_YAW_K_NUM              1
#define F32C_YAW_K_DEN              2
#define F32C_PITCH_K_NUM            1
#define F32C_PITCH_K_DEN            2
```

This means 1 pixel error produces 0.5 x0.1 degrees target increment before step
limiting.

## First Bring-up

1. Flash STM32 without connecting the F32C power first if possible.
2. Confirm MaixCAM2 packets are still decoded; `count` should increase.
3. Connect F32C TTL and power.
4. Hold the target slightly off center and observe direction.
5. If yaw or pitch moves away, flip `F32C_YAW_DIR` or `F32C_PITCH_DIR`.
6. If it oscillates, reduce `*_K_NUM` or increase `*_K_DEN`, or reduce step limit.
7. If it is too slow, increase step limit or gain gradually.

`protocol_test.c` now prints only once every 20 valid packets to reduce debug
UART load during closed-loop control.
