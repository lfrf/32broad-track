#include "f32c_gimbal.h"
#include "protocol_test.h"

#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef huart3;

int32_t Motor1_T_Position = 0;
int32_t Motor2_T_Position = 0;
int32_t Motor1_Current_Position = 0;
int32_t Motor2_Current_Position = 0;
static int16_t Motor1_T_Speed = 0;
static int16_t Motor2_T_Speed = 0;

static uint8_t motor1_Enable_data[5] = {0x7A, 0x01, 0x06, 0x7D, 0x7B};
static uint8_t motor2_Enable_data[5] = {0x7A, 0x02, 0x06, 0x7E, 0x7B};
static uint8_t motor1_Mode_data[7] = {0x7A, 0x01, 0x00, 0x00, 0x00, 0x7B, 0x7B};
static uint8_t motor2_Mode_data[7] = {0x7A, 0x02, 0x00, 0x00, 0x00, 0x78, 0x7B};
static uint8_t motor1_Speed_data[7] = {0x7A, 0x01, 0x01, 0x00, 0x00, 0x00, 0x7B};
static uint8_t motor2_Speed_data[7] = {0x7A, 0x02, 0x01, 0x00, 0x00, 0x00, 0x7B};
static uint8_t motor1_Position_data[9] = {0x7A, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B};
static uint8_t motor2_Position_data[9] = {0x7A, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B};

static uint8_t f32c_bcc(const uint8_t *data, uint8_t len)
{
    uint8_t bcc = 0;
    uint8_t i;

    for(i = 0; i < len; i++){
        bcc ^= data[i];
    }
    return bcc;
}

static int32_t clamp_i32(int32_t v, int32_t low, int32_t high)
{
    if(v < low){
        return low;
    }
    if(v > high){
        return high;
    }
    return v;
}

static int16_t apply_deadzone(int16_t v, int16_t zone)
{
    if((v > -zone) && (v < zone)){
        return 0;
    }
    return v;
}

static void f32c_uart3_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* PB10 -> USART3_TX, PB11 -> USART3_RX. */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

static void f32c_send(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&huart3, data, len, 20);
    if(status != HAL_OK){
        printf("F32C uart3 tx error=%d\r\n", (int)status);
    }
}

static void f32c_send_enable(uint8_t id)
{
    uint8_t *frame = (id == F32C_MOTOR1_ID) ? motor1_Enable_data : motor2_Enable_data;
    frame[1] = id;
    frame[3] = f32c_bcc(frame, 3);
    f32c_send(frame, 5);
}

static void f32c_send_mode(uint8_t id, uint16_t mode)
{
    uint8_t *frame = (id == F32C_MOTOR1_ID) ? motor1_Mode_data : motor2_Mode_data;
    frame[1] = id;
    frame[3] = (uint8_t)(mode >> 8);
    frame[4] = (uint8_t)mode;
    frame[5] = f32c_bcc(frame, 5);
    f32c_send(frame, 7);
}

static void f32c_send_speed(uint8_t id, int16_t speed)
{
    uint8_t *frame = (id == F32C_MOTOR1_ID) ? motor1_Speed_data : motor2_Speed_data;
    frame[1] = id;
    frame[3] = (uint8_t)((uint16_t)speed >> 8);
    frame[4] = (uint8_t)speed;
    frame[5] = f32c_bcc(frame, 5);
    f32c_send(frame, 7);
}

static void F32C_Gimbal_SendSpeedBoth(void)
{
    f32c_send_speed(F32C_MOTOR1_ID, Motor1_T_Speed);
    f32c_send_speed(F32C_MOTOR2_ID, Motor2_T_Speed);
}

static void F32C_Gimbal_SetSpeedTarget(int16_t motor1_rpm, int16_t motor2_rpm)
{
    Motor1_T_Speed = (int16_t)clamp_i32(motor1_rpm, -F32C_YAW_SPEED_LIMIT_RPM, F32C_YAW_SPEED_LIMIT_RPM);
    Motor2_T_Speed = (int16_t)clamp_i32(motor2_rpm, -F32C_PITCH_SPEED_LIMIT_RPM, F32C_PITCH_SPEED_LIMIT_RPM);
}

static void f32c_send_position(uint8_t id, int32_t pos_x10)
{
    uint8_t *frame = (id == F32C_MOTOR1_ID) ? motor1_Position_data : motor2_Position_data;
    uint32_t raw = (uint32_t)pos_x10;

    frame[1] = id;
    frame[3] = (uint8_t)(raw >> 24);
    frame[4] = (uint8_t)(raw >> 16);
    frame[5] = (uint8_t)(raw >> 8);
    frame[6] = (uint8_t)raw;
    frame[7] = f32c_bcc(frame, 7);
    f32c_send(frame, 9);
}

void F32C_Gimbal_SetTarget(int32_t motor1_pos_x10, int32_t motor2_pos_x10)
{
    Motor1_T_Position = clamp_i32(motor1_pos_x10, F32C_YAW_MIN_X10, F32C_YAW_MAX_X10);
    Motor2_T_Position = clamp_i32(motor2_pos_x10, F32C_PITCH_MIN_X10, F32C_PITCH_MAX_X10);
}

void F32C_Gimbal_SendPositionBoth(void)
{
    f32c_send_position(F32C_MOTOR1_ID, Motor1_T_Position);
    HAL_Delay(F32C_INTER_MOTOR_DELAY_MS);
    f32c_send_position(F32C_MOTOR2_ID, Motor2_T_Position);
}

static void F32C_Gimbal_SendPositionBoth_Throttled(uint32_t now)
{
    static uint32_t last_position_send_ms = 0;
    static uint8_t first_send = 1;

    if((first_send == 0) &&
       ((now - last_position_send_ms) < F32C_POSITION_SEND_PERIOD_MS)){
        return;
    }

    first_send = 0;
    last_position_send_ms = now;

    F32C_Gimbal_SendPositionBoth();

#if F32C_DEBUG_PRINT_ENABLE
    printf("POS SEND tgt1=%ld tgt2=%ld\r\n",
           (long)Motor1_T_Position,
           (long)Motor2_T_Position);
#endif
}

static void f32c_hold_target(uint32_t hold_ms)
{
    uint32_t start = HAL_GetTick();

    do {
        F32C_Gimbal_SendPositionBoth();
        HAL_Delay(20);
    } while((HAL_GetTick() - start) < hold_ms);
}

static void f32c_send_raw(uint8_t *data, uint16_t len)
{
    f32c_send(data, len);
}

static void f32c_manual_position_test(void)
{
    static uint8_t x_mode_pos_t[7] = {0x7A, 0x01, 0x00, 0x00, 0x01, 0x7A, 0x7B};
    static uint8_t y_mode_pos_t[7] = {0x7A, 0x02, 0x00, 0x00, 0x01, 0x79, 0x7B};
    static uint8_t x_speed_100[7] = {0x7A, 0x01, 0x01, 0x00, 0x64, 0x1E, 0x7B};
    static uint8_t y_speed_100[7] = {0x7A, 0x02, 0x01, 0x00, 0x64, 0x1D, 0x7B};
    static uint8_t x_pos_360[9] = {0x7A, 0x01, 0x02, 0x00, 0x00, 0x0E, 0x10, 0x67, 0x7B};
    static uint8_t y_pos_360[9] = {0x7A, 0x02, 0x02, 0x00, 0x00, 0x0E, 0x10, 0x64, 0x7B};
    static uint8_t x_pos_0[9] = {0x7A, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x79, 0x7B};
    static uint8_t y_pos_0[9] = {0x7A, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x7A, 0x7B};

    printf("F32C manual position test: exact manual frames\r\n");
    f32c_send_raw(x_mode_pos_t, sizeof(x_mode_pos_t));
    HAL_Delay(100);
    f32c_send_raw(y_mode_pos_t, sizeof(y_mode_pos_t));
    HAL_Delay(100);
    f32c_send_raw(x_speed_100, sizeof(x_speed_100));
    HAL_Delay(100);
    f32c_send_raw(y_speed_100, sizeof(y_speed_100));
    HAL_Delay(300);

    printf("F32C manual: X/Y -> 360deg\r\n");
    f32c_send_raw(x_pos_360, sizeof(x_pos_360));
    HAL_Delay(50);
    f32c_send_raw(y_pos_360, sizeof(y_pos_360));
    HAL_Delay(3000);

    printf("F32C manual: X/Y -> 0deg\r\n");
    f32c_send_raw(x_pos_0, sizeof(x_pos_0));
    HAL_Delay(50);
    f32c_send_raw(y_pos_0, sizeof(y_pos_0));
    HAL_Delay(3000);
}

void F32C_Gimbal_Init(void)
{
    uint8_t wake = 0x00;

    f32c_uart3_init();
    printf("F32C init: USART3 PB10=TX PB11=RX baud=115200\r\n");
    printf("F32C cfg: vision=%d speed_mode=%d manual_pos_test=%d\r\n",
           F32C_VISION_ENABLE, F32C_TRACK_USE_SPEED_MODE, F32C_MANUAL_POSITION_TEST_ENABLE);

    /* Wake up the motor TTL port, then give the F32C controller time to boot. */
    HAL_UART_Transmit(&huart3, &wake, 1, 20);
    HAL_Delay(1500);

    f32c_send_enable(F32C_MOTOR1_ID);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_enable(F32C_MOTOR2_ID);
    HAL_Delay(F32C_CMD_DELAY_MS);

    f32c_send_mode(F32C_MOTOR1_ID, F32C_MODE_POSITION_T);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_MODE_POSITION_T);
    HAL_Delay(F32C_CMD_DELAY_MS);

#if F32C_BOOT_SPEED_TEST_ENABLE
    printf("F32C speed-mode diagnostic: +%d/- %d rpm\r\n", F32C_BOOT_SPEED_TEST_RPM, F32C_BOOT_SPEED_TEST_RPM);
    f32c_send_mode(F32C_MOTOR1_ID, F32C_MODE_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_MODE_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);

    f32c_send_speed(F32C_MOTOR1_ID, F32C_BOOT_SPEED_TEST_RPM);
    f32c_send_speed(F32C_MOTOR2_ID, F32C_BOOT_SPEED_TEST_RPM);
    HAL_Delay(1000);
    f32c_send_speed(F32C_MOTOR1_ID, -F32C_BOOT_SPEED_TEST_RPM);
    f32c_send_speed(F32C_MOTOR2_ID, -F32C_BOOT_SPEED_TEST_RPM);
    HAL_Delay(1000);
    f32c_send_speed(F32C_MOTOR1_ID, 0);
    f32c_send_speed(F32C_MOTOR2_ID, 0);
    HAL_Delay(300);
#endif

#if F32C_TRACK_USE_SPEED_MODE
    printf("F32C switch to speed tracking mode\r\n");
    f32c_send_mode(F32C_MOTOR1_ID, F32C_MODE_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_MODE_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);
    F32C_Gimbal_SetSpeedTarget(0, 0);
    F32C_Gimbal_SendSpeedBoth();
#else
    printf("F32C switch to position mode\r\n");
    f32c_send_mode(F32C_MOTOR1_ID, F32C_MODE_POSITION_T);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_MODE_POSITION_T);
    HAL_Delay(F32C_CMD_DELAY_MS);

    f32c_send_speed(F32C_MOTOR1_ID, F32C_DEFAULT_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_speed(F32C_MOTOR2_ID, F32C_DEFAULT_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);

    F32C_Gimbal_SetTarget(0, 0);
    F32C_Gimbal_SendPositionBoth();
#endif

#if F32C_MANUAL_POSITION_TEST_ENABLE
    f32c_manual_position_test();
#endif

#if F32C_BOOT_SELF_TEST_ENABLE
    printf("F32C boot self-test: motor1 +30/-30, motor2 +30/-30\r\n");
    HAL_Delay(300);

    printf("F32C test motor1 +%d x0.1deg\r\n", F32C_BOOT_SELF_TEST_X10);
    F32C_Gimbal_SetTarget(F32C_BOOT_SELF_TEST_X10, 0);
    f32c_hold_target(1000);

    printf("F32C test motor1 -%d x0.1deg\r\n", F32C_BOOT_SELF_TEST_X10);
    F32C_Gimbal_SetTarget(-F32C_BOOT_SELF_TEST_X10, 0);
    f32c_hold_target(1000);

    printf("F32C test motor2 +%d x0.1deg\r\n", F32C_BOOT_SELF_TEST_X10);
    F32C_Gimbal_SetTarget(0, F32C_BOOT_SELF_TEST_X10);
    f32c_hold_target(1000);

    printf("F32C test motor2 -%d x0.1deg\r\n", F32C_BOOT_SELF_TEST_X10);
    F32C_Gimbal_SetTarget(0, -F32C_BOOT_SELF_TEST_X10);
    f32c_hold_target(1000);

    printf("F32C test back to zero\r\n");
    F32C_Gimbal_SetTarget(0, 0);
    f32c_hold_target(1000);
#endif

    printf("F32C init done\r\n");
}

void F32C_Gimbal_Task(void)
{
    static uint32_t last_control_ms = 0;
    static uint32_t last_seen_count = 0;
    static uint32_t last_seen_ms = 0;
    static uint32_t last_debug_ms = 0;
    uint32_t now = HAL_GetTick();
    uint8_t found;
    int16_t dx;
    int16_t dy;
    int32_t yaw_step;
    int32_t pitch_step;

    if((now - last_control_ms) < F32C_CONTROL_PERIOD_MS){
        return;
    }
    last_control_ms = now;

    if(g_vision_target.valid_count != last_seen_count){
        last_seen_count = g_vision_target.valid_count;
        last_seen_ms = now;
    }

    found = ((g_vision_target.flags & VISION_FLAG_FOUND) != 0) ? 1 : 0;

#if F32C_DEBUG_PRINT_ENABLE
    if((now - last_debug_ms) >= F32C_DEBUG_PRINT_PERIOD_MS){
        last_debug_ms = now;
        printf("GIMBAL vc=%lu found=%u conf=%u raw_dx=%d raw_dy=%d tgt1=%ld tgt2=%ld spd1=%d spd2=%d age=%lu\r\n",
               (unsigned long)g_vision_target.valid_count,
               (unsigned int)found,
               (unsigned int)g_vision_target.confidence,
               (int)g_vision_target.dx,
               (int)g_vision_target.dy,
               (long)Motor1_T_Position,
               (long)Motor2_T_Position,
               (int)Motor1_T_Speed,
               (int)Motor2_T_Speed,
               (unsigned long)(now - last_seen_ms));
    }
#endif

#if F32C_VISION_ENABLE
    if(((now - last_seen_ms) <= F32C_VISION_TIMEOUT_MS) &&
       (found != 0) &&
       (g_vision_target.confidence >= F32C_MIN_CONFIDENCE)){

        dx = apply_deadzone(g_vision_target.dx, F32C_DEADZONE_X_PX);
        dy = apply_deadzone(g_vision_target.dy, F32C_DEADZONE_Y_PX);

#if F32C_TRACK_USE_SPEED_MODE
        yaw_step = ((int32_t)dx * F32C_YAW_SPEED_K_NUM) / F32C_YAW_SPEED_K_DEN;
        pitch_step = ((int32_t)dy * F32C_PITCH_SPEED_K_NUM) / F32C_PITCH_SPEED_K_DEN;

        yaw_step *= F32C_YAW_DIR;
        pitch_step *= F32C_PITCH_DIR;

        F32C_Gimbal_SetSpeedTarget((int16_t)yaw_step, (int16_t)pitch_step);
#else
        yaw_step = ((int32_t)dx * F32C_YAW_K_NUM) / F32C_YAW_K_DEN;
        pitch_step = ((int32_t)dy * F32C_PITCH_K_NUM) / F32C_PITCH_K_DEN;

        yaw_step *= F32C_YAW_DIR;
        pitch_step *= F32C_PITCH_DIR;

        yaw_step = clamp_i32(yaw_step, -F32C_YAW_STEP_LIMIT_X10, F32C_YAW_STEP_LIMIT_X10);
        pitch_step = clamp_i32(pitch_step, -F32C_PITCH_STEP_LIMIT_X10, F32C_PITCH_STEP_LIMIT_X10);

        F32C_Gimbal_SetTarget(Motor1_T_Position + yaw_step,
                              Motor2_T_Position + pitch_step);
#endif
    } else {
#if F32C_TRACK_USE_SPEED_MODE
        F32C_Gimbal_SetSpeedTarget(0, 0);
#endif
    }
#else
#if F32C_TRACK_USE_SPEED_MODE
    F32C_Gimbal_SetSpeedTarget(0, 0);
#endif
#endif

#if F32C_TRACK_USE_SPEED_MODE
    F32C_Gimbal_SendSpeedBoth();
#else
    F32C_Gimbal_SendPositionBoth_Throttled(now);
#endif
}
