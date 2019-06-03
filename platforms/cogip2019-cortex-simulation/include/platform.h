#ifndef PLATFORM_H_
#define PLATFORM_H_

/* Project includes */
#include "platform-common.h"

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

#define HBRIDGE_MOTOR_LEFT  0
#define HBRIDGE_MOTOR_RIGHT 1

#define QDEC_MODE   QDEC_X1
#define QDEC_LEFT_POLARITY      1
#define QDEC_RIGHT_POLARITY     1

#define USART_CONSOLE   USARTC0

static const ctrl_quadpid_parameters_t ctrl_quadpid_params = {
        .linear_speed_pid = {
            .kp = 10,
            .ki = 0.,
            .kd = 0.,
        },
        .angular_speed_pid = {
            .kp = 10,
            .ki = 0.,
            .kd = 0.,
        },
        .linear_pose_pid = {
            .kp = 10,
            .ki = 0.,
            .kd = 0.,
        },
        .angular_pose_pid = {
            .kp = 10,
            .ki = 0.,
            .kd = 0.,
        },

        .min_distance_for_angular_switch = 3 /* mm */,
        .min_angle_for_pose_reached = 2 /* °deg */,
        .regul = CTRL_REGUL_POSE_DIST,
};

void pf_front_cup_take(void);
void pf_front_cup_hold(void);
void pf_front_cup_ramp(void);
void pf_back_cup_take(void);
void pf_back_cup_hold(void);
void pf_back_cup_ramp(void);

void pf_front_ramp_right_drop(void);
void pf_front_ramp_reset(void);
void pf_back_ramp_left_drop(void);
void pf_back_ramp_reset(void);
void pf_back_ramp_left_horiz_for_goldenium(void);
void pf_arms_open(void);
void pf_arms_close(void);
void pf_stop_pumps(void);
void pf_goldenium_hold(void);
void pf_goldenium_take(void);
void pf_goldenium_drop(void);

#endif /* PLATFORM_H_ */
