#include "f32c_gimbal.h"
#include "protocol_test.h"

#include <stdio.h>

static UART_HandleTypeDef huart3;

int32_t Motor1_T_Position = 0;
int32_t Motor2_T_Position = 0;
int32_t Motor1_Current_Position = 0;
int32_t Motor2_Current_Position = 0;
static int16_t Motor1_T_Speed = 0;
static int16_t Motor2_T_Speed = 0;

/* Latest-target overwrite queue:
 * New vision targets overwrite older pending targets.
 * F32C only receives the newest target when the send scheduler is due.
 */
static int32_t g_pending_yaw_x10 = 0;
static int32_t g_pending_pitch_x10 = 0;
static uint8_t g_pending_position_valid = 0;
static uint32_t g_last_position_send_ms = 0;

/* Current track segment supplied by the line-following/main-control board.
 * It is used only for selecting the segmented laser bias.
 */
static uint8_t g_track_zone = F32C_TRACK_ZONE_AB;
static uint32_t g_track_zone_start_ms = 0;

typedef enum {
    F32C_EDGE_STATE_NORMAL = 0,
    F32C_EDGE_STATE_WARN,
    F32C_EDGE_STATE_PANIC,
    F32C_EDGE_STATE_SUSPECT
} F32C_EdgeState;

/* Last reliable vision result. It is used only for edge-area guard.
 * When MaixCAM2 reports found=1 but the center suddenly jumps near the image
 * edge, the gimbal keeps a short safe yaw correction instead of following a
 * possibly wrong corner point immediately.
 */
static uint8_t g_has_last_good_vision = 0;
static int16_t g_last_good_raw_dx = 0;
static int16_t g_last_good_raw_dy = 0;
static int16_t g_last_good_err_x = 0;
static int16_t g_last_good_err_y = 0;
static uint32_t g_last_good_ms = 0;

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

static int32_t f32c_abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t f32c_min_i32(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

static int32_t f32c_slew_i32(int32_t current, int32_t target, int32_t max_delta)
{
    int32_t delta = target - current;

    if(delta > max_delta){
        return current + max_delta;
    }
    if(delta < -max_delta){
        return current - max_delta;
    }
    return target;
}

void F32C_Gimbal_SetTrackZone(uint8_t zone)
{
    zone &= 0x03;

    if(zone != g_track_zone){
        g_track_zone = zone;
        g_track_zone_start_ms = HAL_GetTick();
    }
}

uint8_t F32C_Gimbal_GetTrackZone(void)
{
    return g_track_zone;
}

static void f32c_zone_gpio_init(void)
{
#if F32C_TRACK_ZONE_GPIO_ENABLE
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable both clocks so PA4/PA5 default and common PB alternatives work.
     * If another GPIO port is selected in f32c_gimbal.h, enable its clock here.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = F32C_ZONE0_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(F32C_ZONE0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = F32C_ZONE1_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(F32C_ZONE1_GPIO_PORT, &GPIO_InitStruct);
#endif
}

static uint8_t f32c_read_zone_gpio_raw(void)
{
    uint8_t zone = 0;

#if F32C_TRACK_ZONE_GPIO_ENABLE
    if(HAL_GPIO_ReadPin(F32C_ZONE0_GPIO_PORT, F32C_ZONE0_GPIO_PIN) == GPIO_PIN_SET){
        zone |= 0x01;
    }

    if(HAL_GPIO_ReadPin(F32C_ZONE1_GPIO_PORT, F32C_ZONE1_GPIO_PIN) == GPIO_PIN_SET){
        zone |= 0x02;
    }
#else
    zone = g_track_zone;
#endif

    return zone;
}

static void f32c_update_track_zone_from_gpio(uint32_t now)
{
#if F32C_TRACK_ZONE_GPIO_ENABLE
    static uint32_t last_sample_ms = 0;
    static uint8_t last_raw_zone = F32C_TRACK_ZONE_AB;
    static uint8_t stable_count = 0;
    uint8_t raw_zone;

    if((now - last_sample_ms) < F32C_ZONE_SAMPLE_PERIOD_MS){
        return;
    }
    last_sample_ms = now;

    raw_zone = f32c_read_zone_gpio_raw();

    if(raw_zone == last_raw_zone){
        if(stable_count < F32C_ZONE_STABLE_SAMPLES){
            stable_count++;
        }
    } else {
        last_raw_zone = raw_zone;
        stable_count = 1;
    }

    if(stable_count >= F32C_ZONE_STABLE_SAMPLES){
        F32C_Gimbal_SetTrackZone(raw_zone);
    }
#else
    (void)now;
#endif
}

static int16_t f32c_lerp_i16(int16_t start, int16_t end, uint32_t elapsed_ms, uint32_t total_ms)
{
    int32_t delta;

    if(total_ms == 0){
        return end;
    }

    if(elapsed_ms >= total_ms){
        return end;
    }

    delta = (int32_t)end - (int32_t)start;
    return (int16_t)((int32_t)start + (delta * (int32_t)elapsed_ms) / (int32_t)total_ms);
}

static void f32c_get_current_bias(uint32_t now, int16_t *dx_bias, int16_t *dy_bias)
{
    uint32_t elapsed;

#if F32C_FORCE_TRACK_ZONE_ENABLE
    g_track_zone = F32C_FORCE_TRACK_ZONE;
#endif

    elapsed = now - g_track_zone_start_ms;

    switch(g_track_zone){
    case F32C_TRACK_ZONE_AB:
        *dx_bias = f32c_lerp_i16(F32C_AB_DX_BIAS_START, F32C_AB_DX_BIAS_END,
                                 elapsed, F32C_AB_DURATION_MS);
        *dy_bias = f32c_lerp_i16(F32C_AB_DY_BIAS_START, F32C_AB_DY_BIAS_END,
                                 elapsed, F32C_AB_DURATION_MS);
        break;

    case F32C_TRACK_ZONE_BC:
        *dx_bias = f32c_lerp_i16(F32C_BC_DX_BIAS_START, F32C_BC_DX_BIAS_END,
                                 elapsed, F32C_BC_DURATION_MS);
        *dy_bias = f32c_lerp_i16(F32C_BC_DY_BIAS_START, F32C_BC_DY_BIAS_END,
                                 elapsed, F32C_BC_DURATION_MS);
        break;

    case F32C_TRACK_ZONE_CD:
        *dx_bias = f32c_lerp_i16(F32C_CD_DX_BIAS_START, F32C_CD_DX_BIAS_END,
                                 elapsed, F32C_CD_DURATION_MS);
        *dy_bias = f32c_lerp_i16(F32C_CD_DY_BIAS_START, F32C_CD_DY_BIAS_END,
                                 elapsed, F32C_CD_DURATION_MS);
        break;

    case F32C_TRACK_ZONE_DA:
        *dx_bias = f32c_lerp_i16(F32C_DA_DX_BIAS_START, F32C_DA_DX_BIAS_END,
                                 elapsed, F32C_DA_DURATION_MS);
        *dy_bias = f32c_lerp_i16(F32C_DA_DY_BIAS_START, F32C_DA_DY_BIAS_END,
                                 elapsed, F32C_DA_DURATION_MS);
        break;

    default:
        *dx_bias = F32C_B_DX_BIAS;
        *dy_bias = F32C_B_DY_BIAS;
        break;
    }
}

static uint8_t f32c_is_recenter_guard_active(uint32_t now)
{
#if F32C_RECENTER_GUARD_ENABLE
    uint32_t elapsed = now - g_track_zone_start_ms;

    if(elapsed > F32C_RECENTER_GUARD_MS){
        return 0;
    }

#if F32C_RECENTER_GUARD_APPLY_CD
    if(g_track_zone == F32C_TRACK_ZONE_CD){
        return 1;
    }
#endif

#if F32C_RECENTER_GUARD_APPLY_AB
    if(g_track_zone == F32C_TRACK_ZONE_AB){
        return 1;
    }
#endif

#else
    (void)now;
#endif

    return 0;
}

static F32C_EdgeState f32c_get_edge_state(int16_t raw_dx)
{
#if !F32C_VISION_EDGE_GUARD_ENABLE
    (void)raw_dx;
    return F32C_EDGE_STATE_NORMAL;
#else
    int32_t adx = f32c_abs_i32((int32_t)raw_dx);

    if(adx >= F32C_EDGE_PANIC_X_PX){
        return F32C_EDGE_STATE_PANIC;
    }

    if(adx >= F32C_EDGE_WARN_X_PX){
        return F32C_EDGE_STATE_WARN;
    }

    return F32C_EDGE_STATE_NORMAL;
#endif
}
static int32_t f32c_get_yaw_step_limit_x10(int16_t err_x, int16_t raw_dx, F32C_EdgeState edge_state)
{
    int32_t adx = f32c_abs_i32((int32_t)err_x);
    int32_t limit_x10;

    if(adx >= F32C_YAW_FAR_ERR_PX){
        limit_x10 = F32C_YAW_STEP_LIMIT_FAR_X10;
    } else if(adx >= F32C_YAW_MID_ERR_PX){
        limit_x10 = F32C_YAW_STEP_LIMIT_MID_X10;
    } else {
        limit_x10 = F32C_YAW_STEP_LIMIT_NEAR_X10;
    }

    (void)raw_dx;
    if(edge_state == F32C_EDGE_STATE_PANIC){
        if(limit_x10 < F32C_YAW_STEP_LIMIT_PANIC_X10){
            limit_x10 = F32C_YAW_STEP_LIMIT_PANIC_X10;
        }
    } else if(edge_state == F32C_EDGE_STATE_WARN){
        if(limit_x10 < F32C_YAW_STEP_LIMIT_EDGE_X10){
            limit_x10 = F32C_YAW_STEP_LIMIT_EDGE_X10;
        }
    }

    return limit_x10;
}

static int16_t apply_deadzone_hysteresis(int16_t value,
                                         int16_t stop_zone,
                                         int16_t start_zone,
                                         uint8_t *active)
{
    int32_t magnitude = (value < 0) ? -(int32_t)value : (int32_t)value;

    if(*active != 0){
        if(magnitude <= stop_zone){
            *active = 0;
            return 0;
        }
        return value;
    }

    if(magnitude < start_zone){
        return 0;
    }

    *active = 1;
    return value;
}

static void f32c_calc_laser_error(uint32_t now,
                                  int16_t raw_dx,
                                  int16_t raw_dy,
                                  int16_t *err_x,
                                  int16_t *err_y)
{
    int16_t dx_bias;
    int16_t dy_bias;

#if F32C_USE_B_VISION_BIAS
    f32c_get_current_bias(now, &dx_bias, &dy_bias);
    *err_x = (int16_t)(raw_dx - dx_bias);
    *err_y = (int16_t)(raw_dy - dy_bias);
#else
    *err_x = raw_dx;
    *err_y = raw_dy;
#endif
}

static uint8_t f32c_is_suspicious_edge_frame(int16_t raw_dx, int16_t raw_dy, F32C_EdgeState edge_state)
{
#if !F32C_VISION_EDGE_GUARD_ENABLE
    (void)raw_dx;
    (void)raw_dy;
    (void)edge_state;
    return 0;
#else
    int32_t jump_x;
    int32_t jump_y;

    if(edge_state == F32C_EDGE_STATE_NORMAL){
        return 0;
    }

    if(g_has_last_good_vision == 0){
        return 0;
    }

    jump_x = f32c_abs_i32((int32_t)raw_dx - (int32_t)g_last_good_raw_dx);
    jump_y = f32c_abs_i32((int32_t)raw_dy - (int32_t)g_last_good_raw_dy);

    if(jump_x > F32C_EDGE_JUMP_REJECT_X_PX){
        return 1;
    }

    if(jump_y > F32C_EDGE_JUMP_REJECT_Y_PX){
        return 1;
    }

    if((g_last_good_raw_dx <= -F32C_EDGE_WARN_X_PX) && (raw_dx > 0)){
        return 1;
    }

    if((g_last_good_raw_dx >= F32C_EDGE_WARN_X_PX) && (raw_dx < 0)){
        return 1;
    }

    return 0;
#endif
}
static void f32c_update_last_good_vision(int16_t raw_dx,
                                         int16_t raw_dy,
                                         int16_t err_x,
                                         int16_t err_y,
                                         uint32_t now)
{
    g_last_good_raw_dx = raw_dx;
    g_last_good_raw_dy = raw_dy;
    g_last_good_err_x = err_x;
    g_last_good_err_y = err_y;
    g_last_good_ms = now;
    g_has_last_good_vision = 1;
}

static uint8_t f32c_get_safe_vision_error(int16_t raw_dx,
                                          int16_t raw_dy,
                                          uint32_t now,
                                          int16_t *safe_err_x,
                                          int16_t *safe_err_y,
                                          F32C_EdgeState *edge_state,
                                          uint8_t *used_edge_hold)
{
    int16_t err_x;
    int16_t err_y;
    uint8_t suspicious;

    f32c_calc_laser_error(now, raw_dx, raw_dy, &err_x, &err_y);
    *edge_state = f32c_get_edge_state(raw_dx);
    *used_edge_hold = 0;

    suspicious = f32c_is_suspicious_edge_frame(raw_dx, raw_dy, *edge_state);
    if(suspicious == 0){
        f32c_update_last_good_vision(raw_dx, raw_dy, err_x, err_y, now);
        *safe_err_x = err_x;
        *safe_err_y = err_y;
        return 1;
    }

    *edge_state = F32C_EDGE_STATE_SUSPECT;

    if((g_has_last_good_vision != 0) && ((now - g_last_good_ms) <= F32C_EDGE_HOLD_MS)){
        *safe_err_x = g_last_good_err_x;
        /* Do not chase vertical noise during a suspicious edge frame. */
        *safe_err_y = 0;
        *used_edge_hold = 1;
        return 1;
    }

    return 0;
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

#if F32C_TRACK_USE_SPEED_MODE
static int16_t g_speed_pd_filtered_dx = 0;
static int16_t g_speed_pd_filtered_dy = 0;
static uint32_t g_speed_pd_last_sample_ms = 0;
static uint8_t g_speed_pd_filter_valid = 0;

static void f32c_reset_speed_pd(void)
{
    g_speed_pd_filtered_dx = 0;
    g_speed_pd_filtered_dy = 0;
    g_speed_pd_last_sample_ms = 0;
    g_speed_pd_filter_valid = 0;
    F32C_Gimbal_SetSpeedTarget(0, 0);
}

static void f32c_update_speed_pd(int16_t dx,
                                 int16_t dy,
                                 uint32_t now,
                                 uint8_t used_edge_hold)
{
    int16_t previous_dx = g_speed_pd_filtered_dx;
    int16_t previous_dy = g_speed_pd_filtered_dy;
    uint32_t dt_ms = 0;
    int32_t d_dx_ex10s = 0;
    int32_t d_dy_ex10s = 0;
    int32_t yaw_p;
    int32_t pitch_p;
    int32_t yaw_d;
    int32_t pitch_d;
    int32_t yaw_target;
    int32_t pitch_target;

    if(g_speed_pd_filter_valid == 0){
        g_speed_pd_filtered_dx = dx;
        g_speed_pd_filtered_dy = dy;
        g_speed_pd_filter_valid = 1;
    } else {
        g_speed_pd_filtered_dx =
            (int16_t)(((int32_t)previous_dx *
                       (F32C_SPEED_PD_FILTER_DEN - F32C_SPEED_PD_FILTER_NEW_NUM) +
                       (int32_t)dx * F32C_SPEED_PD_FILTER_NEW_NUM) /
                      F32C_SPEED_PD_FILTER_DEN);
        g_speed_pd_filtered_dy =
            (int16_t)(((int32_t)previous_dy *
                       (F32C_SPEED_PD_FILTER_DEN - F32C_SPEED_PD_FILTER_NEW_NUM) +
                       (int32_t)dy * F32C_SPEED_PD_FILTER_NEW_NUM) /
                      F32C_SPEED_PD_FILTER_DEN);

        dt_ms = now - g_speed_pd_last_sample_ms;
        dt_ms = (uint32_t)clamp_i32((int32_t)dt_ms,
                                    F32C_SPEED_PD_DT_MIN_MS,
                                    F32C_SPEED_PD_DT_MAX_MS);
        d_dx_ex10s = ((int32_t)g_speed_pd_filtered_dx - previous_dx) * 1000 / (int32_t)dt_ms;
        d_dy_ex10s = ((int32_t)g_speed_pd_filtered_dy - previous_dy) * 1000 / (int32_t)dt_ms;
    }
    g_speed_pd_last_sample_ms = now;

    yaw_p = ((int32_t)g_speed_pd_filtered_dx * F32C_YAW_SPEED_PD_KP_NUM) /
            F32C_YAW_SPEED_PD_KP_DEN;
    pitch_p = ((int32_t)g_speed_pd_filtered_dy * F32C_PITCH_SPEED_PD_KP_NUM) /
              F32C_PITCH_SPEED_PD_KP_DEN;
    yaw_d = (d_dx_ex10s * F32C_YAW_SPEED_PD_KD_NUM) /
            F32C_YAW_SPEED_PD_KD_DEN;
    pitch_d = (d_dy_ex10s * F32C_PITCH_SPEED_PD_KD_NUM) /
              F32C_PITCH_SPEED_PD_KD_DEN;

    yaw_d = clamp_i32(yaw_d,
                       -F32C_YAW_SPEED_PD_D_LIMIT_RPM,
                       F32C_YAW_SPEED_PD_D_LIMIT_RPM);
    pitch_d = clamp_i32(pitch_d,
                         -F32C_PITCH_SPEED_PD_D_LIMIT_RPM,
                         F32C_PITCH_SPEED_PD_D_LIMIT_RPM);

    yaw_target = (yaw_p + yaw_d) * F32C_YAW_DIR;
    pitch_target = (pitch_p + pitch_d) * F32C_PITCH_DIR;

    if(used_edge_hold != 0){
        yaw_target = clamp_i32(yaw_target,
                                -F32C_SPEED_PD_EDGE_HOLD_RPM,
                                F32C_SPEED_PD_EDGE_HOLD_RPM);
        pitch_target = 0;
    }

    yaw_target = clamp_i32(yaw_target,
                            -F32C_YAW_SPEED_LIMIT_RPM,
                            F32C_YAW_SPEED_LIMIT_RPM);
    pitch_target = clamp_i32(pitch_target,
                              -F32C_PITCH_SPEED_LIMIT_RPM,
                              F32C_PITCH_SPEED_LIMIT_RPM);
    yaw_target = f32c_slew_i32(Motor1_T_Speed,
                                yaw_target,
                                F32C_YAW_SPEED_PD_SLEW_RPM);
    pitch_target = f32c_slew_i32(Motor2_T_Speed,
                                  pitch_target,
                                  F32C_PITCH_SPEED_PD_SLEW_RPM);

    F32C_Gimbal_SetSpeedTarget((int16_t)yaw_target, (int16_t)pitch_target);
}
#endif

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

static void F32C_Gimbal_LockCurrentAsRelativeZero(void)
{
    /* Software zero only:
     * The operator manually holds the camera at a safe rough pose before boot.
     * After F32C is enabled, the firmware treats that pose as relative 0,0.
     * By default no 0,0 position frame is sent here, because some F32C settings
     * may interpret 0 as an absolute controller origin and move unexpectedly.
     */
    Motor1_T_Position = 0;
    Motor2_T_Position = 0;
    Motor1_Current_Position = 0;
    Motor2_Current_Position = 0;
    g_pending_yaw_x10 = 0;
    g_pending_pitch_x10 = 0;
    g_pending_position_valid = 0;
    g_last_position_send_ms = HAL_GetTick();

#if F32C_BOOT_SEND_ZERO_HOLD_ENABLE
    F32C_Gimbal_SendPositionBoth();
    g_last_position_send_ms = HAL_GetTick();
#endif
}

void F32C_Gimbal_GotoBootRelativeInit(void)
{
    int32_t yaw_delta_x10;
    int32_t pitch_delta_x10;

    yaw_delta_x10 = clamp_i32(F32C_BOOT_YAW_DELTA_X10,
                              -F32C_BOOT_YAW_DELTA_LIMIT_X10,
                              F32C_BOOT_YAW_DELTA_LIMIT_X10);
    pitch_delta_x10 = clamp_i32(F32C_BOOT_PITCH_DELTA_X10,
                                -F32C_BOOT_PITCH_DELTA_LIMIT_X10,
                                F32C_BOOT_PITCH_DELTA_LIMIT_X10);

    if((yaw_delta_x10 != F32C_BOOT_YAW_DELTA_X10) ||
       (pitch_delta_x10 != F32C_BOOT_PITCH_DELTA_X10)){
        printf("F32C boot relative init clamped: cfg_delta=(%ld,%ld) cmd_delta=(%ld,%ld)\r\n",
               (long)F32C_BOOT_YAW_DELTA_X10,
               (long)F32C_BOOT_PITCH_DELTA_X10,
               (long)yaw_delta_x10,
               (long)pitch_delta_x10);
    }

    F32C_Gimbal_LockCurrentAsRelativeZero();
    F32C_Gimbal_SetTarget(yaw_delta_x10, pitch_delta_x10);
    g_pending_position_valid = 0;
    F32C_Gimbal_SendPositionBoth();
    g_last_position_send_ms = HAL_GetTick();
}

void F32C_Gimbal_GotoBPreset(void)
{
    F32C_Gimbal_SetTarget(F32C_B_YAW_X10, F32C_B_PITCH_X10);
    g_pending_position_valid = 0;
    F32C_Gimbal_SendPositionBoth();
    g_last_position_send_ms = HAL_GetTick();
}

static void F32C_Gimbal_QueuePositionTarget(int32_t motor1_pos_x10, int32_t motor2_pos_x10)
{
    g_pending_yaw_x10 = clamp_i32(motor1_pos_x10, F32C_YAW_MIN_X10, F32C_YAW_MAX_X10);
    g_pending_pitch_x10 = clamp_i32(motor2_pos_x10, F32C_PITCH_MIN_X10, F32C_PITCH_MAX_X10);
    g_pending_position_valid = 1;

    /* Do not update Motor*_T_Position until the frame is actually sent.
     * Otherwise the 20ms control loop can accumulate multiple unsent increments
     * and make the software target run ahead of the physical gimbal.
     */
}

static void F32C_Gimbal_SendPendingIfDue(uint32_t now)
{
    if(g_pending_position_valid == 0){
        return;
    }

    if((g_last_position_send_ms != 0) &&
       ((now - g_last_position_send_ms) < F32C_POSITION_SEND_PERIOD_MS)){
        return;
    }

    Motor1_T_Position = g_pending_yaw_x10;
    Motor2_T_Position = g_pending_pitch_x10;

    F32C_Gimbal_SendPositionBoth();

    g_last_position_send_ms = now;
    g_pending_position_valid = 0;

#if F32C_DEBUG_PRINT_ENABLE
    printf("PENDING SEND tgt1=%ld tgt2=%ld\r\n",
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
    f32c_zone_gpio_init();
    g_track_zone = F32C_TRACK_ZONE_AB;
    g_track_zone_start_ms = HAL_GetTick();

    printf("F32C init: USART3 PB10=TX PB11=RX baud=115200\r\n");
    printf("F32C cfg: vision=%d control_mode=%d manual_pos_test=%d boot_relative_init=%d zone_gpio=%d err_ex10=%d edge_guard=%d\r\n",
           F32C_VISION_ENABLE,
           F32C_TRACK_CONTROL_MODE,
           F32C_MANUAL_POSITION_TEST_ENABLE,
           F32C_BOOT_RELATIVE_INIT_ENABLE,
           F32C_TRACK_ZONE_GPIO_ENABLE,
           F32C_VISION_ERROR_UNIT_EX10,
           F32C_VISION_EDGE_GUARD_ENABLE);
    printf("F32C boot relative init: delta=(%ld,%ld) delta_limit=(%ld,%ld) zero_hold=%d B=(%ld,%ld) B_bias=(%d,%d) zone=%u\r\n",
           (long)F32C_BOOT_YAW_DELTA_X10,
           (long)F32C_BOOT_PITCH_DELTA_X10,
           (long)F32C_BOOT_YAW_DELTA_LIMIT_X10,
           (long)F32C_BOOT_PITCH_DELTA_LIMIT_X10,
           F32C_BOOT_SEND_ZERO_HOLD_ENABLE,
           (long)F32C_B_YAW_X10,
           (long)F32C_B_PITCH_X10,
           (int)F32C_B_DX_BIAS,
           (int)F32C_B_DY_BIAS,
           (unsigned int)g_track_zone);

    /* Wake up the motor TTL port, then give the F32C controller time to boot. */
    HAL_UART_Transmit(&huart3, &wake, 1, 20);
    HAL_Delay(1500);

    f32c_send_enable(F32C_MOTOR1_ID);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_enable(F32C_MOTOR2_ID);
    HAL_Delay(F32C_CMD_DELAY_MS);

    f32c_send_mode(F32C_MOTOR1_ID, F32C_TRACK_POSITION_MODE);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_TRACK_POSITION_MODE);
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
    f32c_reset_speed_pd();
    F32C_Gimbal_SendSpeedBoth();
#else
    printf("F32C switch to position mode=%u\r\n", (unsigned int)F32C_TRACK_POSITION_MODE);
    f32c_send_mode(F32C_MOTOR1_ID, F32C_TRACK_POSITION_MODE);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_mode(F32C_MOTOR2_ID, F32C_TRACK_POSITION_MODE);
    HAL_Delay(F32C_CMD_DELAY_MS);

    f32c_send_speed(F32C_MOTOR1_ID, F32C_DEFAULT_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);
    f32c_send_speed(F32C_MOTOR2_ID, F32C_DEFAULT_SPEED);
    HAL_Delay(F32C_CMD_DELAY_MS);

#if F32C_BOOT_RELATIVE_INIT_ENABLE
    /* Boot relative init only:
     * The operator manually sets the rough physical pose before power-on.
     * Firmware treats that pose as software zero, then sends only a small
     * relative yaw/pitch delta. No vision delay/state machine/ready judgment.
     */
    F32C_Gimbal_GotoBootRelativeInit();
#else
    F32C_Gimbal_LockCurrentAsRelativeZero();
#endif
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
    static int16_t last_control_dx = 0;
    static int16_t last_control_dy = 0;
    static uint8_t last_control_valid = 0;
    static int32_t yaw_vel_x10s = 0;
    static int32_t pitch_vel_x10s = 0;
    static int32_t yaw_pos_remainder = 0;
    static int32_t pitch_pos_remainder = 0;
    static uint8_t yaw_deadzone_active = 0;
    static uint8_t pitch_deadzone_active = 0;
    uint32_t now = HAL_GetTick();
    uint8_t found;
    int16_t raw_dx;
    int16_t raw_dy;
    int16_t dx;
    int16_t dy;
    int32_t yaw_step;
    int32_t pitch_step;
    F32C_EdgeState edge_state = F32C_EDGE_STATE_NORMAL;
    uint8_t used_edge_hold = 0;
    uint8_t safe_vision_ok = 0;
    uint8_t new_vision_sample = 0;

    if((now - last_control_ms) < F32C_CONTROL_PERIOD_MS){
        return;
    }
    last_control_ms = now;

    f32c_update_track_zone_from_gpio(now);

    if(g_vision_target.valid_count != last_seen_count){
        last_seen_count = g_vision_target.valid_count;
        last_seen_ms = now;
        new_vision_sample = 1;
    }

    found = ((g_vision_target.flags & VISION_FLAG_FOUND) != 0) ? 1 : 0;
    raw_dx = g_vision_target.dx;
    raw_dy = g_vision_target.dy;

#if F32C_DEBUG_PRINT_ENABLE
    if((now - last_debug_ms) >= F32C_DEBUG_PRINT_PERIOD_MS){
        int16_t dbg_dx_bias;
        int16_t dbg_dy_bias;

        last_debug_ms = now;
        f32c_get_current_bias(now, &dbg_dx_bias, &dbg_dy_bias);
        printf("GIMBAL vc=%lu found=%u conf=%u raw_dx=%d raw_dy=%d zone=%u bias_dx=%d bias_dy=%d tgt1=%ld tgt2=%ld spd1=%d spd2=%d age=%lu edge=%d\r\n",
               (unsigned long)g_vision_target.valid_count,
               (unsigned int)found,
               (unsigned int)g_vision_target.confidence,
               (int)raw_dx,
               (int)raw_dy,
               (unsigned int)g_track_zone,
               (int)dbg_dx_bias,
               (int)dbg_dy_bias,
               (long)Motor1_T_Position,
               (long)Motor2_T_Position,
               (int)Motor1_T_Speed,
               (int)Motor2_T_Speed,
               (unsigned long)(now - last_seen_ms),
               (int)f32c_get_edge_state(raw_dx));
    }
#endif

#if F32C_VISION_ENABLE
#if F32C_TRACK_USE_SPEED_MODE
    if(new_vision_sample == 0){
        if((now - last_seen_ms) <= F32C_SPEED_PD_SAMPLE_HOLD_MS){
            F32C_Gimbal_SendSpeedBoth();
        } else {
            f32c_reset_speed_pd();
            yaw_deadzone_active = 0;
            pitch_deadzone_active = 0;
            last_control_valid = 0;
            F32C_Gimbal_SendSpeedBoth();
        }
        return;
    }
#else
    if((new_vision_sample == 0) &&
       ((now - last_seen_ms) <= F32C_VISION_TIMEOUT_MS)){
        F32C_Gimbal_SendPendingIfDue(now);
        return;
    }
#endif

    if(((now - last_seen_ms) <= F32C_VISION_TIMEOUT_MS) &&
       (found != 0) &&
       (g_vision_target.confidence >= F32C_MIN_CONFIDENCE)){

        safe_vision_ok = f32c_get_safe_vision_error(raw_dx,
                                                    raw_dy,
                                                    now,
                                                    &dx,
                                                    &dy,
                                                    &edge_state,
                                                    &used_edge_hold);

        if(safe_vision_ok != 0){
#if F32C_TRACK_USE_SPEED_MODE
            dx = apply_deadzone_hysteresis(dx,
                                           F32C_SPEED_PD_DEADZONE_X_EX10,
                                           F32C_SPEED_PD_RELEASE_X_EX10,
                                           &yaw_deadzone_active);
            dy = apply_deadzone_hysteresis(dy,
                                           F32C_SPEED_PD_DEADZONE_Y_EX10,
                                           F32C_SPEED_PD_RELEASE_Y_EX10,
                                           &pitch_deadzone_active);

            f32c_update_speed_pd(dx, dy, now, used_edge_hold);
            last_control_dx = dx;
            last_control_dy = dy;
            last_control_valid = 1;
#else
            dx = apply_deadzone_hysteresis(dx,
                                           F32C_DEADZONE_X_PX,
                                           F32C_DEADZONE_RELEASE_X_PX,
                                           &yaw_deadzone_active);
            dy = apply_deadzone_hysteresis(dy,
                                           F32C_DEADZONE_Y_PX,
                                           F32C_DEADZONE_RELEASE_Y_PX,
                                           &pitch_deadzone_active);

            {
                int32_t target_yaw_vel_x10s;
                int32_t target_pitch_vel_x10s;
                int32_t yaw_limit_x10s;
                int32_t pitch_limit_x10s;
                int32_t adx;

                target_yaw_vel_x10s = ((int32_t)dx * F32C_YAW_POS_VEL_K_NUM) / F32C_YAW_POS_VEL_K_DEN;
                target_pitch_vel_x10s = ((int32_t)dy * F32C_PITCH_POS_VEL_K_NUM) / F32C_PITCH_POS_VEL_K_DEN;

                target_yaw_vel_x10s *= F32C_YAW_DIR;
                target_pitch_vel_x10s *= F32C_PITCH_DIR;

                adx = f32c_abs_i32((int32_t)dx);
                if(adx >= F32C_YAW_FAR_ERR_PX){
                    yaw_limit_x10s = F32C_YAW_POS_VEL_LIMIT_FAR_X10S;
                } else if(adx >= F32C_YAW_MID_ERR_PX){
                    yaw_limit_x10s = F32C_YAW_POS_VEL_LIMIT_MID_X10S;
                } else {
                    yaw_limit_x10s = F32C_YAW_POS_VEL_LIMIT_NEAR_X10S;
                }

                if(edge_state == F32C_EDGE_STATE_PANIC){
                    if(yaw_limit_x10s < F32C_YAW_POS_VEL_LIMIT_PANIC_X10S){
                        yaw_limit_x10s = F32C_YAW_POS_VEL_LIMIT_PANIC_X10S;
                    }
                } else if(edge_state == F32C_EDGE_STATE_WARN){
                    if(yaw_limit_x10s < F32C_YAW_POS_VEL_LIMIT_EDGE_X10S){
                        yaw_limit_x10s = F32C_YAW_POS_VEL_LIMIT_EDGE_X10S;
                    }
                }

                if(used_edge_hold != 0){
                    yaw_limit_x10s = f32c_min_i32(yaw_limit_x10s, F32C_EDGE_HOLD_POS_VEL_X10S);
                    target_pitch_vel_x10s = 0;
                }
                if(f32c_is_recenter_guard_active(now) != 0){
                    yaw_limit_x10s = f32c_min_i32(yaw_limit_x10s, F32C_RECENTER_GUARD_POS_VEL_X10S);
                }

                pitch_limit_x10s = F32C_PITCH_POS_VEL_LIMIT_X10S;
                target_yaw_vel_x10s = clamp_i32(target_yaw_vel_x10s, -yaw_limit_x10s, yaw_limit_x10s);
                target_pitch_vel_x10s = clamp_i32(target_pitch_vel_x10s, -pitch_limit_x10s, pitch_limit_x10s);

                if(dx == 0){
                    yaw_vel_x10s = 0;
                    yaw_pos_remainder = 0;
                }
                if(dy == 0){
                    pitch_vel_x10s = 0;
                    pitch_pos_remainder = 0;
                }

#if F32C_ZERO_CROSS_BRAKE_ENABLE
                if(last_control_valid != 0){
                    if(((last_control_dx > 0) && (dx < 0)) ||
                       ((last_control_dx < 0) && (dx > 0))){
                        yaw_vel_x10s = 0;
                        yaw_pos_remainder = 0;
                    }
                    if(((last_control_dy > 0) && (dy < 0)) ||
                       ((last_control_dy < 0) && (dy > 0))){
                        pitch_vel_x10s = 0;
                        pitch_pos_remainder = 0;
                    }
                }
#endif

                yaw_vel_x10s = f32c_slew_i32(yaw_vel_x10s, target_yaw_vel_x10s, F32C_YAW_POS_VEL_SLEW_X10S);
                pitch_vel_x10s = f32c_slew_i32(pitch_vel_x10s, target_pitch_vel_x10s, F32C_PITCH_POS_VEL_SLEW_X10S);

                yaw_pos_remainder += yaw_vel_x10s * (int32_t)F32C_CONTROL_PERIOD_MS;
                pitch_pos_remainder += pitch_vel_x10s * (int32_t)F32C_CONTROL_PERIOD_MS;
                yaw_step = yaw_pos_remainder / 1000;
                pitch_step = pitch_pos_remainder / 1000;
                yaw_pos_remainder %= 1000;
                pitch_pos_remainder %= 1000;

                if((yaw_step != 0) || (pitch_step != 0)){
                    F32C_Gimbal_QueuePositionTarget(Motor1_T_Position + yaw_step,
                                                    Motor2_T_Position + pitch_step);
                }
            }
            last_control_dx = dx;
            last_control_dy = dy;
            last_control_valid = 1;
#endif
        } else {
            last_control_dx = 0;
            last_control_dy = 0;
            last_control_valid = 0;
            yaw_vel_x10s = 0;
            pitch_vel_x10s = 0;
            yaw_pos_remainder = 0;
            pitch_pos_remainder = 0;
            yaw_deadzone_active = 0;
            pitch_deadzone_active = 0;
            g_pending_position_valid = 0;
#if F32C_TRACK_USE_SPEED_MODE
            f32c_reset_speed_pd();
#endif
        }
    } else {
        last_control_dx = 0;
        last_control_dy = 0;
        last_control_valid = 0;
        yaw_vel_x10s = 0;
        pitch_vel_x10s = 0;
        yaw_pos_remainder = 0;
        pitch_pos_remainder = 0;
        yaw_deadzone_active = 0;
        pitch_deadzone_active = 0;
        g_pending_position_valid = 0;
#if F32C_TRACK_USE_SPEED_MODE
        f32c_reset_speed_pd();
#endif
    }
#else
#if F32C_TRACK_USE_SPEED_MODE
    f32c_reset_speed_pd();
#endif
#endif

#if F32C_TRACK_USE_SPEED_MODE
    F32C_Gimbal_SendSpeedBoth();
#else
    F32C_Gimbal_SendPendingIfDue(now);
#endif
}
