#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//#include "console.h"
//#include "encoder.h"
#include "irq.h"
//#include "kos.h"
#include "xtimer.h"
#include "system/log.h"
#include "odometry.h"
#include "platform.h"
#include "trigonometry.h"

#include "controller.h"

uint16_t tempo;

//FIXME: removestub
//#define kos_set_next_schedule_delay_ms(...)
//#define kos_yield(...)

static void set_pose_reached(controller_t *ctrl)
{
	if (ctrl->pose_reached)
		return;

	ctrl->pose_reached = TRUE;
	printf("pose reached\n");
}

/**
 * \fn polar_t compute_error(const pose_t p1, const pose_t p2)
 * \brief compute error between 2 poses
 * \param p1 : setpoint pose
 * \param p2 : measure pose
 * \return distance and angle errors between 2 poses
 */
static polar_t compute_error(controller_t *ctrl,
			 const pose_t pose_order, const pose_t *pose_current)
{
	polar_t error;
	double x, y, O;

	(void)ctrl;

	x = pose_order.x - pose_current->x;
	y = pose_order.y - pose_current->y;

	O = limit_angle_rad(atan2(y, x) - DEG2RAD(pose_current->O));

	error.angle = RAD2DEG(O);
	error.distance = sqrt(square(x) + square(y));

	return error;
}

/**
 * \fn limit_speed_command
 * \brief limit speed command to maximum acceleration and speed setpoint
 * \param command : computed speed by position PID controller
 * \param final_speed : maximum speed
 * \param real_speed
 * \return speed_order
 */
static double limit_speed_command(double command,
				  double final_speed,
				  double real_speed)
{
	/* limit speed command (maximum acceleration) */
	double a = command - real_speed;

	if (a > MAX_ACC)
		command = real_speed + MAX_ACC;

	if (a < -MAX_ACC)
		command = real_speed - MAX_ACC;

	/* limit speed command (speed setpoint) */
	if (command > final_speed)
		command = final_speed;

	if (command < -final_speed)
		command = -final_speed;

	return command;
}

/**
 *
 */
polar_t speed_controller(controller_t *ctrl,
			 polar_t speed_order, polar_t speed_current)
{
	polar_t speed_error;
	polar_t command;
	static uint8_t error_blocking = 0;

	speed_error.distance = speed_order.distance - speed_current.distance;
	speed_error.angle = speed_order.angle - speed_current.angle;

	//double d = fabs(ctrl->linear_speed_pid.previous_error) - fabs(speed_error.distance);
	if (((speed_current.distance < (0.05 * speed_order.distance)) && (ctrl->regul == CTRL_REGUL_POSE_DIST))
		|| ((speed_current.angle < (0.05 * speed_order.angle)) && (ctrl->regul != CTRL_REGUL_POSE_DIST)))
		error_blocking++;
	else
		error_blocking = 0;

	if (error_blocking >= CTRL_BLOCKING_NB_ITERATIONS) {
		command.distance = 0;
		command.angle = 0;
		controller_set_mode(ctrl, CTRL_STATE_BLOCKED);
	}

	command.distance = pid_controller(&ctrl->linear_speed_pid,
					  speed_error.distance);
	command.angle = pid_controller(&ctrl->angular_speed_pid,
					   speed_error.angle);

	return command;
}

polar_t controller_update(controller_t *ctrl,
			  const pose_t *pose_current,
			  polar_t speed_current)
{
	polar_t command 		= {0, 0};
	pose_t	pose_order		= { 0, 0, 0 };
	polar_t speed_order		= { 0, 0 };
	polar_t speed;
	/* ******************** position pid controller ******************** */

	/* compute position error */
	polar_t position_error;

	if (controller_is_pose_reached(&controller)) {
		return command;
	}

	/* get next pose_t to reach */
	pose_order = controller_get_pose_to_reach(&controller);

	pose_order.x *= PULSE_PER_MM;
	pose_order.y *= PULSE_PER_MM;

	/* get speed order */
	speed_order = controller_get_speed_order(&controller);

	position_error = compute_error(ctrl, pose_order, pose_current);

	cons_printf("@c@,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,%+.0f,"
			"%+.0f,%+.0f,"
			"%+.0f,%+.0f,"
			"%+.0f,%+.0f,"
			"%d,"
			"\n",
			  pose_order.x / PULSE_PER_MM,
			  pose_order.y / PULSE_PER_MM,
			  pose_order.O,
			  pose_current->x / PULSE_PER_MM,
			  pose_current->y / PULSE_PER_MM,
			  pose_current->O,
			  position_error.distance / PULSE_PER_MM,
			  position_error.angle,
			  speed_order.distance / PULSE_PER_MM,
			  speed_order.angle,
			  speed_current.distance / PULSE_PER_MM,
			  speed_current.angle,
			  tempo);

	/* position correction */
	if (ctrl->regul != CTRL_REGUL_POSE_ANGL
		&& fabs(position_error.distance) > ctrl->min_distance_for_angular_switch) {

		/* should we go reverse? */
		if (ctrl->allow_reverse && fabs(position_error.angle) > 90) {
			ctrl->in_reverse = TRUE;

			position_error.distance = -position_error.distance;

			if (position_error.angle < 0)
				position_error.angle += 180;
			else
				position_error.angle -= 180;
		} else
			ctrl->in_reverse = FALSE;

		/* if target point direction angle is too important, bot rotates on its starting point */
		if (fabs(position_error.angle) > ctrl->min_angle_for_pose_reached / PULSE_PER_DEGREE) {
			ctrl->regul = CTRL_REGUL_POSE_PRE_ANGL;
			position_error.distance = 0;
			pid_reset(&ctrl->linear_pose_pid);
		}
		else {
			ctrl->regul = CTRL_REGUL_POSE_DIST;
		}
	} else {
		/* orientation correction (position is reached) */
		ctrl->regul = CTRL_REGUL_POSE_ANGL;

		/* final orientation error */
		if (!ctrl->pose_intermediate)
		{
			position_error.angle = limit_angle_deg(pose_order.O - pose_current->O);
		}
		else
		{
			position_error.angle = 0;
		}

		position_error.distance = 0;
		pid_reset(&ctrl->linear_pose_pid);

		/* orientation is reached */
		if (fabs(position_error.angle) < ctrl->min_angle_for_pose_reached / PULSE_PER_DEGREE) {
			position_error.angle = 0;
			pid_reset(&ctrl->angular_pose_pid);

			set_pose_reached(&controller);
			ctrl->regul = CTRL_REGUL_POSE_DIST; //CTRL_REGUL_IDLE;
		}
	}

	position_error.angle *= PULSE_PER_DEGREE;

	/* compute speed command with position pid controller */
	command.distance = pid_controller(&ctrl->linear_pose_pid,
					  position_error.distance);
	command.angle = pid_controller(&ctrl->angular_pose_pid,
					   position_error.angle);


	/* limit speed command */
	speed.distance = limit_speed_command(command.distance,
						 speed_order.distance,
						 speed_current.distance);
	speed.angle = limit_speed_command(command.angle,
					  speed_order.angle,
					  speed_current.angle);

	/* ********************** speed pid controller ********************* */
	return speed_controller(ctrl, speed, speed_current);
}

inline void controller_set_pose_intermediate(controller_t *ctrl, uint8_t intermediate)
{
	ctrl->pose_intermediate = intermediate;
}

inline uint8_t controller_is_in_reverse(controller_t *ctrl)
{
	return ctrl->in_reverse;
}

inline void controller_set_allow_reverse(controller_t *ctrl, uint8_t allow)
{
	ctrl->allow_reverse = allow;
}

inline uint8_t controller_is_pose_reached(controller_t *ctrl)
{
	return ctrl->pose_reached;
}

inline void controller_set_pose_current(controller_t *ctrl, const pose_t pose)
{
	irq_disable();
	ctrl->pose_current = pose;
	irq_enable();
}

inline pose_t controller_get_pose_current(controller_t *ctrl)
{
	pose_t pose_current;

	irq_disable();
	pose_current = ctrl->pose_current;
	irq_enable();

	return pose_current;
}

inline void controller_set_pose_to_reach(controller_t *ctrl, const pose_t pose_order)
{
	irq_disable();
	if (! pose_equal(&ctrl->pose_order, &pose_order)) {
		ctrl->pose_order = pose_order;
		ctrl->pose_reached = FALSE;
	}
	irq_enable();
}

inline pose_t controller_get_pose_to_reach(controller_t *ctrl)
{
	pose_t pose_order;

	irq_disable();
	pose_order = ctrl->pose_order;
	irq_enable();

	return pose_order;
}

inline void controller_set_speed_order(controller_t *ctrl, const polar_t speed_order)
{
	irq_disable();
	ctrl->speed_order = speed_order;
	irq_enable();
}

inline polar_t controller_get_speed_order(controller_t *ctrl)
{
	polar_t	speed_order;

	irq_disable();
	speed_order = ctrl->speed_order;
	irq_enable();

	return speed_order;
}

void controller_set_mode(controller_t *ctrl, controller_mode_id_t new_mode)
{
	ctrl->mode = &controller_modes[new_mode];
	printf("new_mode = %s\n", ctrl->mode->name);
}

void motor_drive(polar_t *command)
{
	/************************ commandes moteur ************************/
	int16_t right_command = (int16_t) (command->distance + command->angle);
	int16_t left_command = (int16_t) (command->distance - command->angle);

	motor_set(0, HBRIDGE_MOTOR_LEFT, (left_command < 0) , abs(left_command));
	motor_set(0, HBRIDGE_MOTOR_RIGHT, (right_command < 0) , abs(right_command));

	log_vect_setvalue(&datalog, LOG_IDX_MOTOR_L, (void *) &left_command);
	log_vect_setvalue(&datalog, LOG_IDX_MOTOR_R, (void *) &right_command);
}

void *task_controller_update(void *arg)
{
	/* bot position on the 'table' (absolute position): */
	polar_t motor_command		= { 0, 0 };
	func_cb_t pfn_evtloop_prefunc  = mach_get_ctrl_loop_pre_pfn();
	func_cb_t pfn_evtloop_postfunc = mach_get_ctrl_loop_post_pfn();

	(void)arg;
	printf("Controller started\n");

	for (;;) {
		xtimer_ticks32_t loop_start_time = xtimer_now();		

		/* Machine specific stuff, if required */
		if (pfn_evtloop_prefunc)
			(*pfn_evtloop_prefunc)();

		if ((controller.mode) && (controller.mode->state_cb))
			controller.mode->state_cb(&controller.pose_current, &motor_command);

		motor_drive(&motor_command);

		/* Machine specific stuff, if required */
		if (pfn_evtloop_postfunc)
			(*pfn_evtloop_postfunc)();

		xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
	}

	return 0;
}

