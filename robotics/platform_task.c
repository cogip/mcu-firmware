#include <stdio.h>
#include <stdlib.h>

#include "planner.h"
#include "platform.h"
#include "platform_task.h"
#include <thread.h>

//FIXME:
#define kos_task_exit()

char controller_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_DEFAULT];
#if defined(MODULE_CALIBRATION)
char calib_thread_stack[THREAD_STACKSIZE_DEFAULT];
#endif /* MODULE_CALIBRATION */

/* Note: last task registered will be the first to be scheduled.
 *	running order can be important in current tasks design */
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
	/*thread_create(calib_thread_stack, sizeof(calib_thread_stack),
				  0, 0,
				  task_calibration_entry, NULL, "calibration");*/
	task_calibration_entry(NULL);
#else
	planner_start_game();
#endif /* MODULE_CALIBRATION */
}
