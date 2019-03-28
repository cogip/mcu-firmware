#ifndef PLATFORM_H_
#define PLATFORM_H_

#include "ctrl.h"
#include "odometry.h"
#include "path.h"
#include "periph/qdec.h"
#include "utils.h"
#include "ctrl/quadpid.h"

#define ROBOT_ID            0
#define PF_START_COUNTDOWN  5

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

#define WHEELS_ENCODER_RESOLUTION   2000
/* WHEELS_PERIMETER = pi*WHEELS_DIAMETER
 * PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
 */
#define PULSE_PER_MM        10.61
/* WHEELS_DISTANCE = WHEELS_DISTANCE_MM * PULSE_PER_MM */
#define WHEELS_DISTANCE     2962.312
/* WHEELS_DISTANCE*2*pi pulses for 360 deg. Thus 51.76 pulses per deg */
#define PULSE_PER_DEGREE    51.70

#define MAX_ACC     10
#define MAX_SPEED   20

#define HBRIDGE_MOTOR_LEFT  0
#define HBRIDGE_MOTOR_RIGHT 1

#define QDEC_MODE   QDEC_X1
#define QDEC_LEFT_POLARITY      1
#define QDEC_RIGHT_POLARITY     1

#define SERVO_ID_VALVE_LAUNCHER 0
#define SERVO_ID_VALVE_RECYCLER 1
#define SERVO_ID_RECYCLER       2
#define SERVO_ID_BEE_L          3
#define SERVO_ID_BEE_R          4
#define SERVO_COUNT             5

#define ADC_RES         ADC_RES_8BIT

#define USART_CONSOLE   USARTC0

#define AVOIDANCE_BORDER_X_MIN  -2000
#define AVOIDANCE_BORDER_X_MAX  2000
#define AVOIDANCE_BORDER_Y_MIN  -2000
#define AVOIDANCE_BORDER_Y_MAX  2000

#define OBSTACLE_BORDER_X_MIN   -2000
#define OBSTACLE_BORDER_X_MAX   2000
#define OBSTACLE_BORDER_Y_MIN   -2000
#define OBSTACLE_BORDER_Y_MAX   2000

#define CTRL_BLOCKING_NB_ITERATIONS 200

path_t *pf_get_path(void);
uint8_t pf_is_game_launched(void);
uint8_t pf_is_camp_left(void);

void pf_ctrl_pre_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_running_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_ctrl_post_stop_cb(pose_t *robot_pose, polar_t* robot_speed, polar_t *motor_command);
void pf_init(void);
void pf_init_tasks(void);

int encoder_read(polar_t *robot_speed);
void encoder_reset(void);

void motor_drive(polar_t *command);

#endif /* PLATFORM_H_ */
