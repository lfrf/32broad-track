#ifndef F32C_GIMBAL_H__
#define F32C_GIMBAL_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define F32C_VISION_ENABLE          1

#define F32C_MOTOR1_ID              1
#define F32C_MOTOR2_ID              2

/* 1: multi-turn position with T trajectory planning. */
#define F32C_MODE_POSITION_T        1

#define F32C_DEFAULT_SPEED          50
#define F32C_CONTROL_PERIOD_MS      20
#define F32C_VISION_TIMEOUT_MS      300

#define F32C_DEADZONE_PX            3
#define F32C_MIN_CONFIDENCE         40

/* Direction depends on camera/gimbal installation. Change signs if it moves away. */
#define F32C_YAW_DIR                1
#define F32C_PITCH_DIR              1

/* Position unit is 0.1 degree. These defaults are deliberately conservative. */
#define F32C_YAW_K_NUM              1
#define F32C_YAW_K_DEN              2
#define F32C_PITCH_K_NUM            1
#define F32C_PITCH_K_DEN            2

#define F32C_YAW_STEP_LIMIT_X10     5
#define F32C_PITCH_STEP_LIMIT_X10   5

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
