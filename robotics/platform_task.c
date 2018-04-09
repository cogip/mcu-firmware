#include <stdio.h>
#include <stdlib.h>

#include "planner.h"
#include "platform.h"
#include "platform_task.h"
#include "xtimer.h"
#include <thread.h>

//FIXME:
#define kos_task_exit()

char controller_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_DEFAULT];

#if defined(MODULE_CALIBRATION)
#define CALIBRATION_BOOT_DELAY 3

char calib_thread_stack[THREAD_STACKSIZE_DEFAULT];
char calib_wait_thread_stack[THREAD_STACKSIZE_DEFAULT];

typedef enum
{
	CALIBRATION_NONE = 0,
	CALIBRATION_READY,
	CALIBRATION_STARTED,
} calibration_state_t;

/* Calibration flag to launch or not calibration thread */
static volatile calibration_state_t calib_flag = 0;

/**
 * @brief Wait for calibration to be started
 *
 * @param[in] arg		thread args (not used)
 *
 * @return				0 for success
 */
static void* calib_wait(void* arg)
{
	(void)arg;

	/* Thread period */
	xtimer_ticks32_t loop_start_time = xtimer_now();
	/* start time and current time to check if CALIBRATION_BOOT_DELAY is exceeded */
	uint32_t start_time = xtimer_now_usec();
	uint32_t current_time = xtimer_now_usec();
	/* Counter for display purpose */
	uint8_t sec_count = 0;

	/* Wait until calibration flag is raised in calling thread or delay is exceeded */
	while ((calib_flag == CALIBRATION_NONE)
		&& (current_time - start_time < CALIBRATION_BOOT_DELAY * US_PER_SEC))
	{
		current_time = xtimer_now_usec();
		xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
		if (current_time > sec_count * US_PER_SEC)
			printf("%u seconds left...\n", CALIBRATION_BOOT_DELAY - sec_count++);
	}

	/* On calibration flag raised, calibration thread is ready */
	if (calib_flag == CALIBRATION_READY)
	{
		puts("Calibration started !");
		/* Up calibration flag to start calibration task */
		calib_flag = CALIBRATION_STARTED;
	}
	/* else launch game */
	else
		planner_start_game();

	return 0;
}
#endif /* MODULE_CALIBRATION */

void mach_tasks_init(void)
{
/* FIXME: Launch calibration task */
	thread_create(controller_thread_stack, sizeof(controller_thread_stack),
				  0, 0,
				  task_controller_update, NULL, "motion_ctrl");
	thread_create(planner_thread_stack, sizeof(planner_thread_stack),
				  0, 0,
				  task_planner, NULL, "game_planner");
#if defined(MODULE_CALIBRATION)
	thread_create(calib_wait_thread_stack, sizeof(calib_wait_thread_stack),
		0, 0,
		calib_wait, NULL, "wait_calibration");

	puts("Press a key to enter calibration...");
	getchar();
	/* Calibration is ready */
	calib_flag = CALIBRATION_READY;
	/* Wait for calibration to be started */
	while (calib_flag < CALIBRATION_STARTED);
	/* Launch calibration function */
	task_calibration_entry(NULL);
#else
	planner_start_game();
#endif /* MODULE_CALIBRATION */
}
