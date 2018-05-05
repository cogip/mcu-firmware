#include "planner.h"

#include <stdio.h>
#include "avoidance.h"
#include "controller.h"
//#include "console.h"
//#include "kos.h"
#include "xtimer.h"
#include "platform.h"
#include "trigonometry.h"
#include "obstacle.h"
#include <irq.h>
#include <periph/adc.h>

//FIXME: remove stub
#define kos_task_exit()
//#define kos_set_next_schedule_delay_ms(...)
//#define kos_yield()

/* Object variables (singleton) */
static uint16_t game_time = 0;
static uint8_t game_started = FALSE;
path_t * path = NULL;
uint8_t in_calibration = FALSE;

/* periodic task */
/* sched period = 20ms -> ticks freq is 1/0.02 = 50 Hz */
#define TASK_PERIOD_MS		(20)

#define TASK_FREQ_HZ		(1000 / TASK_PERIOD_MS)
#define GAME_DURATION_SEC	90
#define GAME_DURATION_TICKS	(GAME_DURATION_SEC * TASK_FREQ_HZ)

inline void increment_current_pose_idx(void)
{
	if (path->current_pose_idx < path->nb_pose - 1)
		path->current_pose_idx += 1;
	else if (path->play_in_loop)
		path->current_pose_idx = 0;
}

static void show_game_time(void)
{
	static uint8_t _secs = TASK_FREQ_HZ;

	if (! --_secs) {
		_secs = TASK_FREQ_HZ;
		printf ("Game time = %d\n",
				game_time / TASK_FREQ_HZ);
	}
}

void planner_start_game(void)
{
	/* TODO: send pose_initial, pose_order & speed_order to controller */
	controller_set_mode(&controller, &controller_modes[CTRL_STATE_INGAME]);
	game_started = TRUE;
}

static int trajectory_get_route_update(const pose_t *robot_pose, pose_t *pose_to_reach)
{
	static pose_t pose_reached;
	pose_t robot_pose_tmp;
	static uint8_t index = 1;
	static int first_boot = 0;
	int test = 0;
	uint8_t need_update = 0;
	(void) robot_pose;
	robot_pose_tmp =  *robot_pose;

	robot_pose_tmp.O /= PULSE_PER_DEGREE;
	robot_pose_tmp.x /= PULSE_PER_MM;
	robot_pose_tmp.y /= PULSE_PER_MM;

	if (first_boot == 0)
	{
		first_boot = 1;
		pose_reached = path->poses[path->current_pose_idx].pos;
		*pose_to_reach = path->poses[path->current_pose_idx].pos;
		set_start_finish(&pose_reached, pose_to_reach);
		if (update_graph() == -1)
			goto trajectory_get_route_update_error;
	}

	if (controller_is_pose_reached(&controller))
	{
		pose_reached = *pose_to_reach;

		if ((pose_to_reach->x == path->poses[path->current_pose_idx].pos.x)
			&& (pose_to_reach->y == path->poses[path->current_pose_idx].pos.y))
		{

			if (!in_calibration)
			{
				if (path->poses[path->current_pose_idx].act)
					path->poses[path->current_pose_idx].act();
				increment_current_pose_idx();
			}
			robot_pose_tmp = pose_reached;
			need_update = 1;
		}
		else
		{
			index++;
		}
	}

#if defined(CONFIG_ANALOG_SENSORS)
	if ((adc_sample(ADC_LINE(0), ADC_RES_10BIT) > 200) && (controller.regul != CTRL_REGUL_POSE_PRE_ANGL))
	{
		add_dyn_obstacle(robot_pose);
		need_update = 1;
	}
#endif

	if (need_update)
	{
		set_start_finish(&robot_pose_tmp, &(path->poses[path->current_pose_idx].pos));
		test = update_graph();
		while ((test < 0) && (!in_calibration))
		{
			if (test == -1)
				increment_current_pose_idx();
			set_start_finish(&robot_pose_tmp, &(path->poses[path->current_pose_idx].pos));
			test = update_graph();
		}
		index = 1;
	}

	*pose_to_reach = avoidance(index);
	if ((pose_to_reach->x == path->poses[path->current_pose_idx].pos.x)
		&& (pose_to_reach->y == path->poses[path->current_pose_idx].pos.y))
	{
		pose_to_reach->O = path->poses[path->current_pose_idx].pos.O;
		controller_set_pose_intermediate(&controller, FALSE);
	}
	else
	{
		controller_set_pose_intermediate(&controller, TRUE);
	}

	return 0;

trajectory_get_route_update_error:
	return -1;
}

void *task_planner(void *arg)
{
	//analog_sensor_zone_t zone;
	func_cb_t pfn_evtloop_end_of_game = mach_get_end_of_game_pfn();
	pose_t	pose_order		= { 0, 0, 0 };
	pose_t	initial_pose		= { 0, 0, 0 };
	polar_t	speed_order		= { 0, 0 };
	const uint8_t camp_yellow	= mach_is_camp_yellow();

	(void)arg;

	printf("Game planner started\n");
	printf("%s camp\n", camp_yellow ? "YELLOW" : "BLUE");

	path = mach_get_path();
	if (!path) {
		printf("machine has no path\n");
		kos_task_exit();
	}

	/* mirror the points in place if selected camp is blue */
	if (!camp_yellow) {
		path->current_pose_idx = path->nb_pose;
		do {
			path->current_pose_idx -= 1;
			path->poses[path->current_pose_idx].pos.x *= -1;
			path->poses[path->current_pose_idx].pos.O = limit_angle_deg(180 - path->poses[path->current_pose_idx].pos.O);
		}
		while (path->current_pose_idx);
	}

	/* object context initialisation */
	path->current_pose_idx = 0;
	initial_pose = path->poses[path->current_pose_idx].pos;
	initial_pose.x *= PULSE_PER_MM;
	initial_pose.y *= PULSE_PER_MM;
	initial_pose.O *= PULSE_PER_DEGREE;
	controller_set_pose_current(&controller, initial_pose);

	for (;;)
	{
		xtimer_ticks32_t loop_start_time = xtimer_now();		

		if (!game_started && !in_calibration)
			goto yield_point;

		//kos_set_next_schedule_delay_ms(20);

		if (!in_calibration) {
			if (pfn_evtloop_end_of_game && game_time >= GAME_DURATION_TICKS)
				(*pfn_evtloop_end_of_game)();

			/* while starter switch is not release we wait */
			if (!mach_is_game_launched())
				goto yield_point;

			if (game_time >= GAME_DURATION_TICKS) {
				cons_printf(">>>>\n");
				controller_set_mode(&controller, &controller_modes[CTRL_STATE_STOP]);
				break;
			}

			if (!game_time) {
				cons_printf("<<<< polar_simu.csv\n");
				cons_printf("@command@,pose_order_x,pose_order_y,pose_order_a,"
						"pose_current_x,pose_current_y,pose_current_a,"
						"position_error_l,position_error_a,"
						"speed_order_l,speed_order_a,"
						"speed_current_l,speed_current_a,"
						"game_time,"
						"\n");
			}

			game_time++;
			show_game_time();
		}


		/* ===== speed ===== */

		/* collision detection */
		/*if (controller_is_in_reverse(&controller))
			zone = AS_ZONE_REAR;
		else
			zone = AS_ZONE_FRONT;

		if (mach_is_zone_obscured(zone)) {
			speed_order.distance = 0;
			speed_order.angle = 0;
		} else {*/
			/* max speed order in pulse_linear per ctrl period (20ms) */
			speed_order.distance = 500;
			/* max speed order in pulse_angular per ctrl period (20ms) */
			speed_order.angle = 250 / 2;
		//}

		controller_set_speed_order(&controller, speed_order);

		/* reverse gear selection is granted per point to reach, in path */
		controller_set_allow_reverse(&controller, path->poses[path->current_pose_idx].allow_reverse);

		pose_t pose_current = controller.pose_current;

		/* ===== position ===== */
		if (trajectory_get_route_update(&pose_current, &pose_order) == -1)
		{
			controller_set_mode(&controller, &controller_modes[CTRL_STATE_STOP]);
		}

		controller_set_pose_to_reach(&controller, pose_order);

yield_point:
		//kos_yield();
		xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
	}

//	controller.mode = CTRL_STATE_INGAME;
//	cons_printf("calibration ended\n");
	kos_task_exit();

	return 0;
}

