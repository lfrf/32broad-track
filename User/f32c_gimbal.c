#include "f32c_gimbal.h"
#include "protocol_test.h"

#include <string.h>

static UART_HandleTypeDef huart3;

int32_t Motor1_T_Position = 0;
int32_t Motor2_T_Position = 0;
int32_t Motor1_Current_Position = 0;
int32_t Motor2_Current_Position = 0;

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
    HAL_UART_Transmit(&huart3, data, len, 20);
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
    f32c_send_position(F32C_MOTOR2_ID, Motor2_T_Position);
}

void F32C_Gimbal_Init(void)
{
    uint8_t wake = 0x00;

    f32c_uart3_init();

    /* Wake up the motor TTL port, then give the F32C controller time to boot. */
    HAL_UART_Transmit(&huart3, &wake, 1, 20);
    HAL_Delay(1500);

    f32c_send_enable(F32C_MOTOR1_ID);
    HAL_Delay(2);
    f32c_send_enable(F32C_MOTOR2_ID);
    HAL_Delay(2);

    f32c_send_mode(F32C_MOTOR1_ID, F32C_MODE_POSITION_T);
    HAL_Delay(2);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_MODE_POSITION_T);
    HAL_Delay(2);

    f32c_send_speed(F32C_MOTOR1_ID, F32C_DEFAULT_SPEED);
    HAL_Delay(10);
    f32c_send_speed(F32C_MOTOR2_ID, F32C_DEFAULT_SPEED);
    HAL_Delay(10);

    F32C_Gimbal_SetTarget(0, 0);
    F32C_Gimbal_SendPositionBoth();
}

void F32C_Gimbal_Task(void)
{
    static uint32_t last_control_ms = 0;
    static uint32_t last_seen_count = 0;
    static uint32_t last_seen_ms = 0;
    uint32_t now = HAL_GetTick();
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

#if F32C_VISION_ENABLE
    if(((now - last_seen_ms) <= F32C_VISION_TIMEOUT_MS) &&
       ((g_vision_target.flags & VISION_FLAG_FOUND) != 0) &&
       (g_vision_target.confidence >= F32C_MIN_CONFIDENCE)){

        dx = apply_deadzone(g_vision_target.dx, F32C_DEADZONE_PX);
        dy = apply_deadzone(g_vision_target.dy, F32C_DEADZONE_PX);

        yaw_step = ((int32_t)dx * F32C_YAW_K_NUM) / F32C_YAW_K_DEN;
        pitch_step = ((int32_t)dy * F32C_PITCH_K_NUM) / F32C_PITCH_K_DEN;

        yaw_step *= F32C_YAW_DIR;
        pitch_step *= F32C_PITCH_DIR;

        yaw_step = clamp_i32(yaw_step, -F32C_YAW_STEP_LIMIT_X10, F32C_YAW_STEP_LIMIT_X10);
        pitch_step = clamp_i32(pitch_step, -F32C_PITCH_STEP_LIMIT_X10, F32C_PITCH_STEP_LIMIT_X10);

        F32C_Gimbal_SetTarget(Motor1_T_Position + yaw_step,
                              Motor2_T_Position + pitch_step);
    }
#endif

    F32C_Gimbal_SendPositionBoth();
}


