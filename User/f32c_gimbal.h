#ifndef F32C_GIMBAL_H__
#define F32C_GIMBAL_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define F32C_VISION_ENABLE          1
#define F32C_DEBUG_PRINT_ENABLE     0
#define F32C_DEBUG_PRINT_PERIOD_MS  500
#define F32C_TRACK_MODE_POSITION_VELOCITY 0
#define F32C_TRACK_MODE_EX10_SPEED_PD      1

/* Change this one line to switch the vision outer-loop controller. */
#define F32C_TRACK_CONTROL_MODE F32C_TRACK_MODE_EX10_SPEED_PD

#if F32C_TRACK_CONTROL_MODE == F32C_TRACK_MODE_EX10_SPEED_PD
#define F32C_TRACK_USE_SPEED_MODE 1
#elif F32C_TRACK_CONTROL_MODE == F32C_TRACK_MODE_POSITION_VELOCITY
#define F32C_TRACK_USE_SPEED_MODE 0
#else
#error Unsupported_F32C_TRACK_CONTROL_MODE
#endif

/* MaixCAM2 binary_v1 still uses the dx/dy fields, but in this mode they carry
 * laser-plane error in 0.1 mm: dx=ex10, dy=ey10. They are no longer
 * image pixel errors.
 */
#define F32C_VISION_ERROR_UNIT_EX10 1
#define F32C_VISION_EDGE_GUARD_ENABLE 0

#define F32C_MOTOR1_ID              1
#define F32C_MOTOR2_ID              2

#define F32C_MODE_SPEED             0
/* 1: multi-turn position with T trajectory planning.
 * 3: multi-turn position direct mode. The F32C manual recommends direct mode
 *    when target position is changed at high frequency.
 */
#define F32C_MODE_POSITION_T        1
#define F32C_MODE_POSITION_DIRECT   3
#define F32C_TRACK_POSITION_MODE    F32C_MODE_POSITION_DIRECT

#define F32C_DEFAULT_SPEED          180
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
#define F32C_POSITION_SEND_PERIOD_MS 20
#define F32C_INTER_MOTOR_DELAY_MS    5

/* Vision error deadzone. In F32C_VISION_ERROR_UNIT_EX10 mode these values
 * are 0.1 mm, so 50 means 5.0 mm.
 */
#define F32C_DEADZONE_PX            3
#define F32C_DEADZONE_X_PX          80
#define F32C_DEADZONE_Y_PX          90
#define F32C_DEADZONE_RELEASE_X_PX  120
#define F32C_DEADZONE_RELEASE_Y_PX  140
#define F32C_MIN_CONFIDENCE         40

/* Direction depends on camera/gimbal installation.
 * If your final hardware behaves opposite, change the corresponding value.
 */
#define F32C_YAW_DIR                1
#define F32C_PITCH_DIR              -1

/* Position unit is 0.1 degree. */
#define F32C_YAW_K_NUM              1
#define F32C_YAW_K_DEN              2

/* Pitch is deliberately slower/stabler than yaw to reduce vertical oscillation. */
#define F32C_PITCH_K_NUM            1
#define F32C_PITCH_K_DEN            4
/* Error-rate feedforward. It reacts to target motion before pure position error
 * fully builds up, improving follow response without changing final setpoint.
 */
#define F32C_YAW_D_NUM              1
#define F32C_YAW_D_DEN              2
#define F32C_PITCH_D_NUM            1
#define F32C_PITCH_D_DEN            4
#define F32C_YAW_D_STEP_LIMIT_X10   8
#define F32C_PITCH_D_STEP_LIMIT_X10 4

#define F32C_YAW_STEP_LIMIT_X10     8
/* Segmented yaw step limits for faster large-error recovery without large
 * horizontal oscillation near the center.
 */
#define F32C_YAW_STEP_LIMIT_NEAR_X10   3
#define F32C_YAW_STEP_LIMIT_MID_X10    9
#define F32C_YAW_STEP_LIMIT_FAR_X10    18
#define F32C_YAW_MID_ERR_PX            200
#define F32C_YAW_FAR_ERR_PX            500

/* Edge-aware tracking guard.
 * raw_dx is used for edge-state judgment because it represents the real target
 * position in the camera frame. Bias-corrected err_x is still used as the
 * laser aiming error.
 */
#define F32C_EDGE_WARN_X_PX             40
#define F32C_EDGE_PANIC_X_PX            58
#define F32C_EDGE_LOST_X_PX             70
#define F32C_YAW_STEP_LIMIT_EDGE_X10    20
#define F32C_YAW_STEP_LIMIT_PANIC_X10   25

/* Reject suspicious center jumps when the target is already near the image edge.
 * This prevents the gimbal from blindly following a wrongly detected corner.
 */
#define F32C_EDGE_JUMP_REJECT_X_PX      25
#define F32C_EDGE_JUMP_REJECT_Y_PX      20
#define F32C_EDGE_HOLD_MS               120
#define F32C_EDGE_HOLD_STEP_X10         10

/* Recenter guard: when the car exits a turn and the chassis heading snaps back,
 * raw_dx may briefly jump toward the frame edge. During the first short window
 * after entering CD or AB, cap yaw step to avoid edge-boost overreaction.
 */
#define F32C_RECENTER_GUARD_ENABLE       1
#define F32C_RECENTER_GUARD_MS           350
#define F32C_RECENTER_GUARD_YAW_LIMIT_X10 10
#define F32C_RECENTER_GUARD_APPLY_AB     1
#define F32C_RECENTER_GUARD_APPLY_CD     1

#define F32C_PITCH_STEP_LIMIT_X10   7
/* If the visual error changes sign, the laser has crossed the target center.
 * Shrink the next position increment to avoid repeatedly overshooting.
 */
#define F32C_ZERO_CROSS_BRAKE_ENABLE 1
#define F32C_ZERO_CROSS_BRAKE_DIV    3

/* Direct ex10/ey10 speed-PD controller. Error unit is 0.1 mm. */
#define F32C_SPEED_PD_DEADZONE_X_EX10       40
#define F32C_SPEED_PD_DEADZONE_Y_EX10       55
#define F32C_SPEED_PD_RELEASE_X_EX10        65
#define F32C_SPEED_PD_RELEASE_Y_EX10        80
#define F32C_SPEED_PD_FILTER_NEW_NUM         1
#define F32C_SPEED_PD_FILTER_DEN             2
#define F32C_YAW_SPEED_PD_KP_NUM             3
#define F32C_YAW_SPEED_PD_KP_DEN           100
#define F32C_PITCH_SPEED_PD_KP_NUM           2
#define F32C_PITCH_SPEED_PD_KP_DEN         100
#define F32C_YAW_SPEED_PD_KD_NUM             1
#define F32C_YAW_SPEED_PD_KD_DEN           500
#define F32C_PITCH_SPEED_PD_KD_NUM           1
#define F32C_PITCH_SPEED_PD_KD_DEN         650
#define F32C_YAW_SPEED_PD_D_LIMIT_RPM        6
#define F32C_PITCH_SPEED_PD_D_LIMIT_RPM      4
#define F32C_YAW_SPEED_PD_SLEW_RPM           4
#define F32C_PITCH_SPEED_PD_SLEW_RPM         3
#define F32C_YAW_SPEED_LIMIT_RPM             22
#define F32C_PITCH_SPEED_LIMIT_RPM           14
#define F32C_SPEED_PD_EDGE_HOLD_RPM           6
#define F32C_SPEED_PD_SAMPLE_HOLD_MS        100
#define F32C_SPEED_PD_DT_MIN_MS              10
#define F32C_SPEED_PD_DT_MAX_MS             150

/* Position-mode velocity outer loop.
 * The F32C stays in position mode for holding torque, while vision error first
 * becomes a target angular velocity and is then integrated into position.
 * Unit: x10 degree/s, i.e. 100 means 10.0 deg/s.
 */
#define F32C_YAW_POS_VEL_K_NUM      1
#define F32C_YAW_POS_VEL_K_DEN      3
#define F32C_PITCH_POS_VEL_K_NUM    1
#define F32C_PITCH_POS_VEL_K_DEN    4
#define F32C_YAW_POS_VEL_LIMIT_NEAR_X10S 15
#define F32C_YAW_POS_VEL_LIMIT_MID_X10S  40
#define F32C_YAW_POS_VEL_LIMIT_FAR_X10S  80
#define F32C_YAW_POS_VEL_LIMIT_EDGE_X10S 1000
#define F32C_YAW_POS_VEL_LIMIT_PANIC_X10S 1200
#define F32C_PITCH_POS_VEL_LIMIT_X10S 25
#define F32C_YAW_POS_VEL_SLEW_X10S  5
#define F32C_PITCH_POS_VEL_SLEW_X10S 3
#define F32C_EDGE_HOLD_POS_VEL_X10S 500
#define F32C_RECENTER_GUARD_POS_VEL_X10S 500

#define F32C_YAW_MIN_X10            (-3800)
#define F32C_YAW_MAX_X10            (3800)
#define F32C_PITCH_MIN_X10          (-2700)
#define F32C_PITCH_MAX_X10          (2700)

/* ===================== Calibration values =====================
 *
 * Unit of *_YAW_X10 / *_PITCH_X10:
 *   0.1 degree. Example: 184 means 18.4 degrees.
 *
 * In F32C_VISION_ERROR_UNIT_EX10 mode, MaixCAM2 packet dx/dy are ex10/ey10:
 *   laser error in 0.1 mm. Old *_DX_BIAS / *_DY_BIAS pixel biases must stay
 *   disabled unless they are recalibrated in ex10/ey10 units.
 */

/* ===================== Boot relative init =====================
 *
 * New startup method:
 *   1. Before power-on, manually hold/move the camera to a safe rough pose.
 *   2. After F32C is enabled and switched to position mode, the firmware treats
 *      the current physical pose as the software zero reference.
 *   3. It sends only a small relative yaw/pitch offset below.
 *   4. Normal vision tracking then naturally continues from that relative target.
 *
 * This avoids relying on F32C's absolute zero and avoids commanding a large
 * pitch lift such as 160 degrees during boot.
 */
#define F32C_BOOT_RELATIVE_INIT_ENABLE      1

/* Relative boot offsets from the manually held power-on pose.
 * Unit: 0.1 degree.
 * Example: -200 means -20.0 degrees, 50 means +5.0 degrees.
 * Tune these two values on-site.
 */
#define F32C_BOOT_YAW_DELTA_X10             (-250)
#define F32C_BOOT_PITCH_DELTA_X10           0

/* Safety clamps for the boot relative offset itself.
 * These are delta limits, not absolute joint limits.
 * They prevent accidental large relative boot moves if DELTA values are edited
 * incorrectly. Final targets are still also clamped by F32C_YAW/PITCH_MIN/MAX.
 */
#define F32C_BOOT_YAW_DELTA_LIMIT_X10       300
#define F32C_BOOT_PITCH_DELTA_LIMIT_X10     300

/* Optional lock command. Keep this 0 unless you have verified that sending
 * position 0,0 after power-on holds the current physical pose instead of
 * returning to the controller's absolute zero.
 */
#define F32C_BOOT_SEND_ZERO_HOLD_ENABLE     0

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
#define F32C_USE_B_VISION_BIAS      0
#define F32C_B_DX_BIAS              (-2)
#define F32C_B_DY_BIAS              (-6)

/* ===================== Track zone input and segmented bias =====================
 *
 * Two-line absolute zone code from the line-following/main-control board:
 *   ZONE1 ZONE0 = 00: A->B straight
 *   ZONE1 ZONE0 = 01: B->C first turn
 *   ZONE1 ZONE0 = 10: C->D straight
 *   ZONE1 ZONE0 = 11: D->A second turn
 *
 * STM32 wiring default:
 *   PA4 = ZONE0 low bit, connected to main-control PB17
 *   PA5 = ZONE1 high bit, connected to main-control PB18
 *   GND must be shared.
 */
#define F32C_TRACK_ZONE_AB          0
#define F32C_TRACK_ZONE_BC          1
#define F32C_TRACK_ZONE_CD          2
#define F32C_TRACK_ZONE_DA          3

#define F32C_TRACK_ZONE_GPIO_ENABLE 1
#define F32C_ZONE0_GPIO_PORT        GPIOA
#define F32C_ZONE0_GPIO_PIN         GPIO_PIN_4
#define F32C_ZONE1_GPIO_PORT        GPIOA
#define F32C_ZONE1_GPIO_PIN         GPIO_PIN_5

/* Debounce the 2-bit code so that brief transition states do not switch bias. */
#define F32C_ZONE_SAMPLE_PERIOD_MS  20
#define F32C_ZONE_STABLE_SAMPLES    2

/* Force-zone mode for calibration without connecting the main-control board. */
#define F32C_FORCE_TRACK_ZONE_ENABLE 0
#define F32C_FORCE_TRACK_ZONE        F32C_TRACK_ZONE_AB

/* Segment duration for start->end bias interpolation. */
#define F32C_AB_DURATION_MS         3200
#define F32C_BC_DURATION_MS         4470
#define F32C_CD_DURATION_MS         4500
#define F32C_DA_DURATION_MS         5340

/* Segmented laser aiming bias.
 * Current values are calibrated from field tests. Tune DY_BIAS first if
 * vertical laser error remains, then tune DX_BIAS.
 */
#define F32C_AB_DX_BIAS_START       (-6)
#define F32C_AB_DY_BIAS_START       (-12)
#define F32C_AB_DX_BIAS_END         (-2)
#define F32C_AB_DY_BIAS_END         (-8)

#define F32C_BC_DX_BIAS_START       (-3)
#define F32C_BC_DY_BIAS_START       (-7)
#define F32C_BC_DX_BIAS_END         (-2)
#define F32C_BC_DY_BIAS_END         (-5)

#define F32C_CD_DX_BIAS_START       (-2)
#define F32C_CD_DY_BIAS_START       (-5)
#define F32C_CD_DX_BIAS_END         (-2)
#define F32C_CD_DY_BIAS_END         (-6)

#define F32C_DA_DX_BIAS_START       (-2)
#define F32C_DA_DY_BIAS_START       (-6)
#define F32C_DA_DX_BIAS_END         (-1)
#define F32C_DA_DY_BIAS_END         (-6)

extern int32_t Motor1_T_Position;
extern int32_t Motor2_T_Position;
extern int32_t Motor1_Current_Position;
extern int32_t Motor2_Current_Position;

void F32C_Gimbal_Init(void);
void F32C_Gimbal_Task(void);
void F32C_Gimbal_SetTarget(int32_t motor1_pos_x10, int32_t motor2_pos_x10);
void F32C_Gimbal_SendPositionBoth(void);
void F32C_Gimbal_GotoBootRelativeInit(void);
void F32C_Gimbal_GotoBPreset(void);
void F32C_Gimbal_SetTrackZone(uint8_t zone);
uint8_t F32C_Gimbal_GetTrackZone(void);

#endif
