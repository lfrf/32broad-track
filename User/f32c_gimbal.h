#ifndef F32C_GIMBAL_H__
#define F32C_GIMBAL_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define F32C_VISION_ENABLE          1
#define F32C_DEBUG_PRINT_ENABLE     1
#define F32C_DEBUG_PRINT_PERIOD_MS  500
#define F32C_TRACK_USE_SPEED_MODE   0

#define F32C_MOTOR1_ID              1
#define F32C_MOTOR2_ID              2


#define F32C_MODE_SPEED             0
/* 1: multi-turn position with T trajectory planning. */
#define F32C_MODE_POSITION_T        1

#define F32C_DEFAULT_SPEED          120
#define F32C_BOOT_SELF_TEST_ENABLE  0
#define F32C_BOOT_SPEED_TEST_ENABLE 0
#define F32C_MANUAL_POSITION_TEST_ENABLE 0
#define F32C_BOOT_SELF_TEST_X10     300
#define F32C_BOOT_SPEED_TEST_RPM    60
#define F32C_CMD_DELAY_MS           30
#define F32C_CONTROL_PERIOD_MS      20
#define F32C_VISION_TIMEOUT_MS      300

/* Test-only change:
 * Keep the vision/control calculation period at 20ms, but throttle F32C
 * position-frame output to 50ms and insert a small gap between motor frames.
 */
#define F32C_POSITION_SEND_PERIOD_MS 50
#define F32C_INTER_MOTOR_DELAY_MS    10

#define F32C_DEADZONE_PX            3
#define F32C_MIN_CONFIDENCE         40

/* Direction depends on camera/gimbal installation. Change signs if it moves away. */
#define F32C_YAW_DIR                -1
#define F32C_PITCH_DIR              -1

/* Position unit is 0.1 degree. These defaults are deliberately conservative. */
#define F32C_YAW_K_NUM              1
#define F32C_YAW_K_DEN              1
#define F32C_PITCH_K_NUM            1
#define F32C_PITCH_K_DEN            1

#define F32C_YAW_STEP_LIMIT_X10     8
#define F32C_PITCH_STEP_LIMIT_X10   8

#define F32C_YAW_SPEED_K_NUM        1
#define F32C_YAW_SPEED_K_DEN        2
#define F32C_PITCH_SPEED_K_NUM      1
#define F32C_PITCH_SPEED_K_DEN      2
#define F32C_YAW_SPEED_LIMIT_RPM    35
#define F32C_PITCH_SPEED_LIMIT_RPM  25

#define F32C_YAW_MIN_X10            (-900)
#define F32C_YAW_MAX_X10            (900)
#define F32C_PITCH_MIN_X10          (-450)
#define F32C_PITCH_MAX_X10          (450)

extern int32_t Motor1_T_Position;
extern int32_t Motor2_T_Position;
extern int32_t Motor1_Current_Position;
extern int32_t Motor2_Current_Position;

void F32C_Gimbal_Init(void);
void F32C_Gimbal_Task(void);
void F32C_Gimbal_SetTarget(int32_t motor1_pos_x10, int32_t motor2_pos_x10);
void F32C_Gimbal_SendPositionBoth(void);

#endif
