#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdint.h>

#include "odometry.h"
#include "pid.h"

typedef void (*state_cb_t)(pose_t *, polar_t *);

typedef struct {
	char *name;
	state_cb_t state_cb;
} controller_mode_t;

typedef enum {
	CTRL_REGUL_IDLE = 0,
	CTRL_REGUL_POSE_DIST,
	CTRL_REGUL_POSE_ANGL,
	//CTRL_REGUL_SPEED, /* time for actions */
} controller_regul_t;

typedef struct {
	double wheels_distance;

	PID_t linear_speed_pid;
	PID_t angular_speed_pid;
	PID_t linear_pose_pid;
	PID_t angular_pose_pid;

	/* Distance approximation to switch to angular correction */
	uint16_t min_distance_for_angular_switch;

	/* Angle approximation to switch to position reached state */
	uint16_t min_angle_for_pose_reached;


	/* Dynamics variables */
	controller_mode_t mode;

	pose_t pose_order;
	polar_t	speed_order;
	uint8_t allow_reverse;

	controller_regul_t regul;
	uint8_t pose_reached;
	uint8_t pose_intermediate;
	uint8_t in_reverse;
} controller_t;


polar_t speed_controller(controller_t *ctrl,
			 polar_t speed_setpoint, polar_t real_speed);

polar_t controller_update(controller_t *ctrl,
			  pose_t pose_setpoint, pose_t current_pose,
			  polar_t speed_setpoint, polar_t current_speed);

void controller_set_pose_intermediate(controller_t *ctrl, uint8_t intermediate);
uint8_t controller_is_in_reverse(controller_t *ctrl);
void controller_set_allow_reverse(controller_t *ctrl, uint8_t allow);

uint8_t controller_is_pose_reached(controller_t *ctrl);
void controller_set_pose_to_reach(controller_t *ctrl, const pose_t pose_order);
pose_t controller_get_pose_to_reach(controller_t *ctrl);

void controller_set_speed_order(controller_t *ctrl, const polar_t speed_order);
polar_t controller_get_speed_order(controller_t *ctrl);

void controller_set_mode(controller_t *ctrl, controller_mode_t new_mode);

void *task_controller_update(void *arg);

#if defined(CONFIG_CALIBRATION)
void controller_enter_calibration(void);
#endif

#endif /* CONTROLLER_H_ */
