#include "controller.h"
#include "platform.h"
#include "system/log.h"

//FIXME: removestub
#define hbridge_engine_update(...)
//#define log_vect_setvalue(...)
//#define kos_set_next_schedule_delay_ms(...)
//#define kos_yield(...)
//#define encoder_reset()

#define PWM_RANGE 500

extern uint16_t tempo;

void ctrl_state_calib_mode1_cb(pose_t *robot_pose, polar_t *motor_command)
{
	(void)robot_pose;
	polar_t	robot_speed		= { 0, 0 };
	/*
	 * First calibration test:
	 * Perform two PWM sweeps to characterize encoders.
	 */

	/* first entry, we reset the datalog */
	if (!tempo)
		log_vect_reset(&datalog, "cal1_up",
				LOG_IDX_SPEED_L,
				LOG_IDX_SPEED_R,
				LOG_IDX_MOTOR_L,
				LOG_IDX_MOTOR_R,
				-1);
	/*
	 * Two ramps :
	 * 1. [-pwm ... +pwm] for 400 cycles (0.02 = 8s)
	 * 2. [+pwm ... -pwm] for 400 cycles
	 */
	motor_command->angle = 0;
	if (tempo < 50)
		motor_command->distance = -PWM_RANGE;
	else if (tempo >= 50 && tempo < 400 - 50)
		motor_command->distance = (int16_t)((double)(tempo - 50) * 2*PWM_RANGE/300.) - PWM_RANGE;
	else if (tempo >= 400 - 50 && tempo < 400 + 50)
		motor_command->distance = PWM_RANGE;
	else if (tempo >= 450 && tempo < 800 - 50)
		motor_command->distance = -((int16_t)((double)(tempo - 450) * 2*PWM_RANGE/300.) - PWM_RANGE);
	else if (tempo >= 800 - 50)
		motor_command->distance = -PWM_RANGE;

	motor_drive(motor_command);

	/* catch speed */
	//TODO: control return
	encoder_read(&robot_speed);

	log_vect_display_line(&datalog);

	tempo ++;
	if (tempo == 400) {
		//motor_command->distance = 0;
		//motor_command->angle = 0;
		//motor_drive(motor_command);

		log_vect_display_last_line(&datalog);
		/* prepare next log */
		log_vect_reset(&datalog, "cal1_down",
				LOG_IDX_SPEED_L,
				LOG_IDX_SPEED_R,
				LOG_IDX_MOTOR_L,
				LOG_IDX_MOTOR_R,
				-1);
	} else if (tempo == 800) {
		motor_command->distance = 0;
		motor_command->angle = 0;
		motor_drive(motor_command);

		log_vect_display_last_line(&datalog);

		controller_set_mode(&controller, &controller_modes[CTRL_STATE_STOP]);
		tempo = 0;
	}
}

void ctrl_state_calib_mode2_cb(pose_t *robot_pose, polar_t *motor_command)
{
	(void)robot_pose;
	polar_t	robot_speed		= { 0, 0 };
	polar_t	speed_order		= { 0, 0 };
	/*
	 * Second calibration test:
	 * Perform a speed command to tune Kp, Ki (& Kd).
	 */

	/* first entry, we reset the datalog */
	if (!tempo) {
		encoder_reset();
		log_vect_reset(&datalog, "cal2",
				LOG_IDX_ROBOT_SPEED_D,
				/*LOG_IDX_ROBOT_SPEED_A,*/
				LOG_IDX_SPEED_ORDER_D,
				/*LOG_IDX_SPEED_ORDER_A,*/
				LOG_IDX_SPEED_L,
				LOG_IDX_SPEED_R,
				-1);
	}
	/*
	 * t[0s..1s] : speed is set to 0
	 * t[1s..7s] : speed is set to full
	 * t[7s..8s] : speed is set to 0
	 */
	if (tempo < 50)
		speed_order.distance = 0;
	else if (tempo >= 50 && tempo < 400 - 50)
		speed_order.distance = 15;
	else if (tempo >= 400 - 50)
		speed_order.distance = 0;

	speed_order.angle = 0;

	log_vect_setvalue(&datalog, LOG_IDX_SPEED_ORDER_D, (void *) &speed_order.distance);

	/* catch speed */
	// TODO: control return
	encoder_read(&robot_speed);
	*motor_command = speed_controller(&controller,
					 speed_order,
					 robot_speed);

	log_vect_setvalue(&datalog, LOG_IDX_ROBOT_SPEED_D, (void *) &robot_speed.distance);

	motor_drive(motor_command);

	log_vect_display_line(&datalog);

	tempo ++;
	if (tempo == 400) {
		motor_command->distance = 0;
		motor_command->angle = 0;
		motor_drive(motor_command);

		log_vect_display_last_line(&datalog);

		controller_set_mode(&controller, &controller_modes[CTRL_STATE_STOP]);
		tempo = 0;
	}
}

void ctrl_state_calib_mode3_cb(pose_t *robot_pose, polar_t *motor_command)
{
	(void)robot_pose;
	polar_t	robot_speed		= { 0, 0 };
	polar_t	speed_order		= { 0, 0 };
	/*
	 * Third calibration test:
	 * Perform a speed command to tune Kp, Ki (& Kd).
	 */

	/* first entry, we reset the datalog */
	if (!tempo) {
		encoder_reset();
		log_vect_reset(&datalog, "cal3",
				LOG_IDX_ROBOT_SPEED_D,
				/*LOG_IDX_ROBOT_SPEED_A,*/
				LOG_IDX_SPEED_ORDER_D,
				/*LOG_IDX_SPEED_ORDER_A,*/
				LOG_IDX_MOTOR_L,
				LOG_IDX_MOTOR_R,
				-1);
	}
	/*
	 * t[0s..2s] : angular speed is set to +15
	 * t[2s..4s] : angular speed is set to -30
	 * t[4s..6s] : angular speed is set to +30
	 * t[6s..8s] : angular speed is set to -15
	 */
	if (tempo < 100)
		speed_order.angle = +15;
	else if (tempo >= 100 && tempo < 200)
		speed_order.angle = -30;
	else if (tempo >= 200 && tempo < 300)
		speed_order.angle = +30;
	else if (tempo >= 300 && tempo < 400)
		speed_order.angle = -15;

	speed_order.distance = 0;

	log_vect_setvalue(&datalog, LOG_IDX_SPEED_ORDER_D, (void *) &speed_order.distance);

	/* catch speed */
	// TODO: control return
	encoder_read(&robot_speed);
	*motor_command = speed_controller(&controller,
					 speed_order,
					 robot_speed);

	log_vect_setvalue(&datalog, LOG_IDX_ROBOT_SPEED_D, (void *) &robot_speed.distance);

	motor_drive(motor_command);

	log_vect_display_line(&datalog);

	tempo ++;
	if (tempo == 400) {
		motor_command->distance = 0;
		motor_command->angle = 0;
		motor_drive(motor_command);

		log_vect_display_last_line(&datalog);

		controller_set_mode(&controller, &controller_modes[CTRL_STATE_STOP]);
		tempo = 0;
	}
}

static PID_t *controller_get_pid_from_idx(const uint8_t i)
{
	switch(i) {
	case 0:
		return &controller.linear_speed_pid;
	case 1:
		return &controller.angular_speed_pid;
	case 2:
		return &controller.linear_pose_pid;
	case 3:
		return &controller.angular_pose_pid;
	default:
		return NULL;
	}
}

//static const char *controller_get_pid_name_from_idx(const uint8_t i)
//{
//	switch(i) {
//	case 0:
//		return "linear_speed_pid";
//	case 1:
//		return "angular_speed_pid";
//	case 2:
//		return "linear_pose_pid";
//	case 3:
//		return "angular_pose_pid";
//	default:
//		return NULL;
//	}
//}

static void controller_calibration_usage(const uint8_t pid_idx)
{
	cons_printf("\n>>> Entering controller calibration mode\n\n");

	cons_printf("\t'1' to launch motor sweep [-pwm..+pwm] (out: cal1*.csv)\n");
	cons_printf("\t       this will calibrate : encoders & pwm ranges\n");
	cons_printf("\t'2' to launch speed control loop only (out: cal2.csv)\n");
	cons_printf("\t       this will calibrate : Kp, Ki & (Kd) for speed PID\n");
	cons_printf("\t'3' to launch angular control loop only (out: cal3.csv)\n");
	cons_printf("\n");
	cons_printf("\t'v' to dump all PID values\n");
	cons_printf("\t'n' to select next PID to be tuned\n");
	cons_printf("\t'b' to select prev PID to be tuned\n");
	cons_printf("\n");
(void)pid_idx;
//	cons_printf("\%s:\t"
//	       "Kp = %+.2f\tKi = %+.2f\tKd = %+.2f\n",
//	       controller_get_pid_name_from_idx(pid_idx),
//	       controller_get_pid_from_idx(pid_idx)->kp,
//	       controller_get_pid_from_idx(pid_idx)->ki,
//	       controller_get_pid_from_idx(pid_idx)->kd);
	cons_printf("\t'p' to tune Kp\n");
	cons_printf("\t'i' to tune Ki\n");
	cons_printf("\t'd' to tune Kd\n");
	cons_printf("\n");
	cons_printf("\t'h' to display this help\n");
	cons_printf("\t'q' to quit\n");
	cons_printf("\n");
}

static void scanf_update_val (const char *var_name, double *var)
{
	cons_printf("%s = %+.2f\tenter new value: ", var_name , *var);
	cons_scanf("%lf", var);
	cons_printf("new %s = %+.2f\n", var_name, *var);
}

void controller_enter_calibration(void)
{
	int c, i;
	uint8_t quit = 0;
	uint8_t pid_idx = 0;
	PID_t *cur_pid = controller_get_pid_from_idx(pid_idx);

	controller_calibration_usage(pid_idx);

	while (!quit) {

		/* display prompt */
		cons_printf("$ ");

		/* wait for command */
		c = cons_getchar();
		cons_printf("%c\n", c);

		switch (c) {
		case '1':
			controller_set_mode(&controller, &controller_modes[CTRL_STATE_CALIB_MODE1]);
			break;
		case '2':
			controller_set_mode(&controller, &controller_modes[CTRL_STATE_CALIB_MODE2]);
			break;
		case '3':
			controller_set_mode(&controller, &controller_modes[CTRL_STATE_CALIB_MODE3]);
			break;
		case 'p':
			scanf_update_val("Kp", &cur_pid->kp);
			break;
		case 'i':
			scanf_update_val("Ki", &cur_pid->ki);
			break;
		case 'd':
			scanf_update_val("Kd", &cur_pid->kd);
			break;
		case 'n':
			pid_idx += 1;
			pid_idx %= 4;
			cur_pid = controller_get_pid_from_idx(pid_idx);
			controller_calibration_usage(pid_idx);
			break;
		case 'b':
			if (!pid_idx)
				pid_idx = 3;
			else
				pid_idx -= 1;
			cur_pid = controller_get_pid_from_idx(pid_idx);
			controller_calibration_usage(pid_idx);
			break;
		case 'v':
			for (i = 0; i < 4; i++) {
//				cons_printf("\%s:\t"
//				       "Kp = %+.2f\tKi = %+.2f\tKd = %+.2f\n",
//				       controller_get_pid_name_from_idx(i),
//				       controller_get_pid_from_idx(i)->kp,
//				       controller_get_pid_from_idx(i)->ki,
//				       controller_get_pid_from_idx(i)->kd);
			}
			break;
		case 'h':
			controller_calibration_usage(pid_idx);
			break;
		case 'q':
			quit = 1;
			break;
		default:
			cons_printf("\n");
			break;
		}
	}
}
