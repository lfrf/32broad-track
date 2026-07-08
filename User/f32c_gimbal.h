#ifndef F32C_GIMBAL_H__
#define F32C_GIMBAL_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define F32C_VISION_ENABLE          1
#define F32C_DEBUG_PRINT_ENABLE     0
#define F32C_DEBUG_PRINT_PERIOD_MS  500
#define F32C_TRACK_USE_SPEED_MODE   0

#define F32C_MOTOR1_ID              1
#define F32C_MOTOR2_ID              2

#define F32C_MODE_SPEED             0
/* 1: multi-turn position with T trajectory planning. */
#define F32C_MODE_POSITION_T        1

#define F32C_DEFAULT_SPEED          160
#define F32C_BOOT_SELF_TEST_ENABLE  0
#define F32C_BOOT_SPEED_TEST_ENABLE 0
#define F32C_MANUAL_POSITION_TEST_ENABLE 0
#define F32C_BOOT_SELF_TEST_X10     300
#define F32C_BOOT_SPEED_TEST_RPM    60
#define F32C_CMD_DELAY_MS           30
#define F32C_CONTROL_PERIOD_MS      20
#define F32C_VISION_TIMEOUT_MS      300

/* Keep the previous verified fix:
 * control still runs at 20ms, but F32C position frames are sent at <= 50ms,
 * with a short gap between motor1 and motor2 frames.
 */
#define F32C_POSITION_SEND_PERIOD_MS 30
#define F32C_INTER_MOTOR_DELAY_MS    5

/* Legacy common deadzone kept for compatibility/reference, but the task now uses
 * separated X/Y deadzones below.
 */
#define F32C_DEADZONE_PX            3
#define F32C_DEADZONE_X_PX          8
#define F32C_DEADZONE_Y_PX          10
#define F32C_MIN_CONFIDENCE         40

/* Direction depends on camera/gimbal installation.
 * If your final hardware behaves opposite, change the corresponding value.
 */
#define F32C_YAW_DIR                1
#define F32C_PITCH_DIR              -1

/* Position unit is 0.1 degree. */
#define F32C_YAW_K_NUM              1
#define F32C_YAW_K_DEN              1

/* Pitch is deliberately slower/stabler than yaw to reduce vertical oscillation. */
#define F32C_PITCH_K_NUM            1
#define F32C_PITCH_K_DEN            3

#define F32C_YAW_STEP_LIMIT_X10     8
/* Segmented yaw step limits for faster large-error recovery without large
 * horizontal oscillation near the center.
 */
#define F32C_YAW_STEP_LIMIT_NEAR_X10   5
#define F32C_YAW_STEP_LIMIT_MID_X10    8
#define F32C_YAW_STEP_LIMIT_FAR_X10    10
#define F32C_YAW_MID_ERR_PX            16
#define F32C_YAW_FAR_ERR_PX            32

#define F32C_PITCH_STEP_LIMIT_X10   3

#define F32C_YAW_SPEED_K_NUM        1
#define F32C_YAW_SPEED_K_DEN        2
#define F32C_PITCH_SPEED_K_NUM      1
#define F32C_PITCH_SPEED_K_DEN      2
#define F32C_YAW_SPEED_LIMIT_RPM    35
#define F32C_PITCH_SPEED_LIMIT_RPM  25

#define F32C_YAW_MIN_X10            (-1800)
#define F32C_YAW_MAX_X10            (1800)
#define F32C_PITCH_MIN_X10          (-450)
#define F32C_PITCH_MAX_X10          (450)

/* ===================== Calibration values =====================
 *
 * Unit of *_YAW_X10 / *_PITCH_X10:
 *   0.1 degree. Example: 184 means 18.4 degrees.
 *
 * Unit of *_DX_BIAS / *_DY_BIAS:
 *   pixel error from MaixCAM2 vision packet.
 *
 * Fill these values after using the motor-only calibration firmware.
 */

/* Startup preset:
 * Used only after power-on, so the camera/gimbal first points roughly toward
 * the target paper and vision tracking can start reliably.
 */
#define F32C_INIT_YAW_X10           0
#define F32C_INIT_PITCH_X10         10

/* B-point preset:
 * Optional. This is the yaw/pitch angle when laser hits the target center at B.
 * The current main loop will not automatically jump to this value unless you
 * call F32C_Gimbal_GotoBPreset() from your car-state logic.
 */
#define F32C_B_YAW_X10              (-180)
#define F32C_B_PITCH_X10            50

/* B-point vision bias:
 * Critical for final laser hit accuracy when using vision tracking.
 * If laser hits the center at B while the vision log shows raw_dx/raw_dy not zero,
 * put those raw values here.
 */
#define F32C_USE_B_VISION_BIAS      1
#define F32C_B_DX_BIAS              2
#define F32C_B_DY_BIAS              (-18)

extern int32_t Motor1_T_Position;
extern int32_t Motor2_T_Position;
extern int32_t Motor1_Current_Position;
extern int32_t Motor2_Current_Position;

void F32C_Gimbal_Init(void);
void F32C_Gimbal_Task(void);
void F32C_Gimbal_SetTarget(int32_t motor1_pos_x10, int32_t motor2_pos_x10);
void F32C_Gimbal_SendPositionBoth(void);
void F32C_Gimbal_GotoBPreset(void);

#endif
