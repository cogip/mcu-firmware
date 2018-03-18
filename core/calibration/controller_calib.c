#include "controller.h"

#if defined(CONFIG_CALIBRATION)
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
			controller_set_mode(&controller, CTRL_STATE_CALIB_MODE1);
			break;
		case '2':
			controller_set_mode(&controller, CTRL_STATE_CALIB_MODE2);
			break;
		case '3':
			controller_set_mode(&controller, CTRL_STATE_CALIB_MODE3);
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
#endif /* CONFIG_CALIBRATION */

