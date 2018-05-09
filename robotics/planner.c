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
#define TASK_PERIOD_MS		(200)

#define TASK_FREQ_HZ		(1000 / TASK_PERIOD_MS)
#define GAME_DURATION_SEC	100
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
	controller_set_mode(&controller, CTRL_STATE_INGAME);
	game_started = TRUE;
}

static int trajectory_get_route_update(const pose_t *robot_pose, pose_t *pose_to_reach, polar_t *speed_order)
{
	static pose_t pose_reached;
	path_pose_t *current_path_pos = path_get_current_path_pos(path);
	pose_t robot_pose_tmp;
	static uint8_t index = 1;
	static int first_boot = 0;
	int test = 0;
	int control_loop = 0;
	uint8_t need_update = 0;
	(void) robot_pose;
	robot_pose_tmp =  *robot_pose;

	robot_pose_tmp.O /= PULSE_PER_DEGREE;
	robot_pose_tmp.x /= PULSE_PER_MM;
	robot_pose_tmp.y /= PULSE_PER_MM;

	if (first_boot == 0)
	{
		first_boot = 1;
		pose_reached = current_path_pos->pos;
		*pose_to_reach = current_path_pos->pos;
		set_start_position_finish_position(&pose_reached, pose_to_reach);
		if (update_graph() == -1)
			goto trajectory_get_route_update_error;
	}

	if (controller_is_pose_reached(&controller))
	{
		pose_reached = *pose_to_reach;

		if ((pose_to_reach->x == current_path_pos->pos.x)
			&& (pose_to_reach->y == current_path_pos->pos.y))
		{

			if (!in_calibration)
			{
#ifndef BOARD_NATIVE
				if (current_path_pos->act)
					current_path_pos->act();
#endif
				increment_current_pose_idx();
				current_path_pos = path_get_current_path_pos(path);
			}
			robot_pose_tmp = pose_reached;
			need_update = 1;
		}
		else
		{
			index++;
		}
	}

	reset_dyn_polygons();

#if defined(CONFIG_ANALOG_SENSORS)
	if(controller.regul != CTRL_REGUL_POSE_PRE_ANGL) {
		for (int i = 0; i < ana_sensors.sensors_nb; i++) {
			double dist = analog_sensor_check_obstacle(&ana_sensors, i);
			if (dist < AS_DIST_LIMIT) {
				if (add_dyn_obstacle(robot_pose, &ana_sensors.sensors[i], dist*10) == 0)
					need_update = 1;
			}
		}
	}
#endif

	if (controller.mode == &controller_modes[CTRL_STATE_BLOCKED])
	{
		increment_current_pose_idx();
		controller_set_mode(&controller, CTRL_STATE_INGAME);
		need_update = 1;
	}

	if (need_update)
	{
		set_start_position_finish_position(&robot_pose_tmp, &(current_path_pos->pos));
		test = update_graph();

		control_loop = path->nb_pose;
		while ((test < 0) && (!in_calibration) && (control_loop-- > 0))
		{
			if (test == -1) {
				increment_current_pose_idx();
				current_path_pos = path_get_current_path_pos(path);
			}
			set_start_position_finish_position(&robot_pose_tmp, &(current_path_pos->pos));
			test = update_graph();
		}
		if (control_loop == 0)
		{
			goto trajectory_get_route_update_error;
		}
		index = 1;
	}

	*pose_to_reach = avoidance(index);
	if ((pose_to_reach->x == current_path_pos->pos.x)
		&& (pose_to_reach->y == current_path_pos->pos.y))
	{
		pose_to_reach->O = current_path_pos->pos.O;
		controller_set_pose_intermediate(&controller, FALSE);
	}
	else
	{
		/* Update speed order to max speed defined value in the new point to reach */
		if (current_path_pos->max_speed <= MAX_SPEED)
			speed_order->distance = current_path_pos->max_speed;
		else
			speed_order->distance = MAX_SPEED;
		speed_order->angle = speed_order->distance / 2;
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
	const uint8_t camp_left	= mach_is_camp_left();
	path_pose_t *current_path_pos = NULL;

	(void)arg;

	printf("Game planner started\n");
	/* 2018: Camp left is orange, right is green */
	printf("%s camp\n", camp_left ? "LEFT" : "RIGHT");

	path = mach_get_path();
	if (!path) {
		printf("machine has no path\n");
		kos_task_exit();
	}

	/* mirror the points in place if selected camp is right */
	if (!camp_left)
		path_horizontal_mirror_all_pos(path);

	/* object context initialisation */
	path->current_pose_idx = 0;
	current_path_pos = path_get_current_path_pos(path);
	initial_pose = current_path_pos->pos;
	initial_pose.x *= PULSE_PER_MM;
	initial_pose.y *= PULSE_PER_MM;
	initial_pose.O *= PULSE_PER_DEGREE;
	controller_set_pose_current(&controller, initial_pose);

	uint32_t game_start_time = xtimer_now_usec();

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
			{
				game_start_time = xtimer_now_usec();
				goto yield_point;
			}

			if (xtimer_now_usec() - game_start_time >= GAME_DURATION_SEC * US_PER_SEC) {
				cons_printf(">>>>\n");
				controller_set_mode(&controller, CTRL_STATE_STOP);
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

		current_path_pos = path_get_current_path_pos(path);
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
			//speed_order.distance = current_path_pos->max_speed;
			/* max speed order in pulse_angular per ctrl period (20ms) */
			//speed_order.angle = speed_order.distance / 2;
		//}

		/* Update speed order to max speed defined value in the new point to reach */
		if (current_path_pos->max_speed <= MAX_SPEED)
			speed_order.distance = current_path_pos->max_speed;
		else
			speed_order.distance = MAX_SPEED;
		speed_order.angle = speed_order.distance / 2;

		/* reverse gear selection is granted per point to reach, in path */
		controller_set_allow_reverse(&controller, current_path_pos->allow_reverse);

		pose_t pose_current = controller.pose_current;

		/* ===== position ===== */
		if (trajectory_get_route_update(&pose_current, &pose_order, &speed_order) == -1)
		{
			controller_set_mode(&controller, CTRL_STATE_STOP);
		}

		controller_set_speed_order(&controller, speed_order);

		controller_set_pose_to_reach(&controller, pose_order);

yield_point:
		//kos_yield();
		xtimer_periodic_wakeup(&loop_start_time, TASK_PERIOD_MS * US_PER_MS);
	}

//	controller.mode = CTRL_STATE_INGAME;
//	cons_printf("calibration ended\n");
	kos_task_exit();

	return 0;
}

