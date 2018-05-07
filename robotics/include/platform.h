#ifndef PLATFORM_H_
#define PLATFORM_H_

#include "controller.h"
#include "analog_sensor.h"
#include "system/log.h"
#include "odometry.h"
#include "path.h"
#include "periph/qdec.h"
#include "actuators/sd21.h"
#include "utils.h"

/*
 * Machine parameters
 */

/* To be computed :
 *  - PULSE_PER_MM		: Number of pulses per mm of coding wheel
 *  - WHEELS_DISTANCE		: Distance between coding wheels in pulses
 *  - PULSE_PER_DEGREE		: Number of pulses per degree of coding wheel
 *
 * Must be known :
 *  - WHEELS_DIAMETER		: Coding wheel diameter
 *  - WHEELS_DISTANCE_MM	: Distance between coding wheels in mm
 *
 * Must be known and defined :
 *  - WHEELS_ENCODER_RESOLUTION	: Number of pulses by turn of coding wheels
 */

#define WHEELS_ENCODER_RESOLUTION	2000
/* WHEELS_PERIMETER = pi*WHEELS_DIAMETER
 * PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
 */
#define PULSE_PER_MM			10.61
/* WHEELS_DISTANCE = WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define WHEELS_DISTANCE			2962.312
/* WHEELS_DISTANCE*2*pi pulses for 360 deg. Thus 51.76 pulses per deg */
#define PULSE_PER_DEGREE		51.70

#define MAX_ACC				25
#define MAX_SPEED			250

#define HBRIDGE_MOTOR_LEFT		0
#define HBRIDGE_MOTOR_RIGHT		1

#if defined(BOARD_NATIVE)
#define QDEC_MODE				QDEC_X1
#else
#define QDEC_MODE				QDEC_X4
#endif
#define QDEC_LEFT_POLARITY		-1
#define QDEC_RIGHT_POLARITY		1

#define SERVO_ID_VALVE_LAUNCHER		0
#define SERVO_ID_VALVE_RECYCLER		1
#define SERVO_ID_RECYCLER		2
#define SERVO_ID_BEE_L			3
#define SERVO_ID_BEE_R			4
#define SERVO_COUNT			5

#define ADC_RES					ADC_RES_10BIT

#define USART_CONSOLE			USARTC0

#define AVOIDANCE_BORDER_X_MIN    -1318
#define AVOIDANCE_BORDER_X_MAX    1318
#define AVOIDANCE_BORDER_Y_MIN    182
#define AVOIDANCE_BORDER_Y_MAX    1818

#define CTRL_BLOCKING_NB_ITERATIONS	100

#if defined(MODULE_CALIBRATION)
typedef enum {
	CTRL_STATE_CALIB_MODE1 = CTRL_STATE_INGAME + 1,
	CTRL_STATE_CALIB_MODE2,
	CTRL_STATE_CALIB_MODE3,
} controller_mode_id_calib_t;
#endif /* MODULE_CALIBRATION */

extern analog_sensors_t ana_sensors;
extern qdec_t encoders[];
extern sd21_t sd21;
extern controller_t controller;
extern controller_mode_t controller_modes[];

extern datalog_t datalog;

func_cb_t mach_get_ctrl_loop_pre_pfn(void);
func_cb_t mach_get_ctrl_loop_post_pfn(void);
func_cb_t mach_get_end_of_game_pfn(void);
path_t * mach_get_path(void);
//uint8_t mach_is_zone_obscured(analog_sensor_zone_t zone);
uint8_t mach_is_game_launched(void);
uint8_t mach_is_camp_left(void);

void ctrl_state_stop_cb(pose_t *robot_pose, polar_t *motor_command);
void ctrl_state_idle_cb(pose_t *robot_pose, polar_t *motor_command);
void ctrl_state_ingame_cb(pose_t *robot_pose, polar_t *motor_command);
#if defined(MODULE_CALIBRATION)
void ctrl_state_calib_mode1_cb(pose_t *robot_pose, polar_t *motor_command);
void ctrl_state_calib_mode2_cb(pose_t *robot_pose, polar_t *motor_command);
void ctrl_state_calib_mode3_cb(pose_t *robot_pose, polar_t *motor_command);

void controller_enter_calibration(void);
void mach_check_calibration_mode(void);
#endif /* MODULE_CALIBRATION */

void mach_setup(void);
void mach_sched_init(void);
void mach_sched_run(void);

int encoder_read(polar_t *robot_speed);
void encoder_reset(void);

#endif /* PLATFORM_H_ */
