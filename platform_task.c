#include <stdio.h>
#include <stdlib.h>

//#include "console.h"
//#include "controller.h"
//#include "encoder.h"
//#include "hbridge.h"
//#include "kos.h"
//#include "log.h"
//#include "mcurses.h"
#include "core/planner.h"
#include "platform.h"
#include "platform_task.h"
#include "thread.h"
//#include "usart.h"
//#include "qdec.h"

//FIXME:
#define kos_task_exit()

#if defined(CONFIG_CALIBRATION)
static void mach_calibration_usage(void)
{
	cons_printf("\n>>> Entering calibration mode\n\n");

#if defined(CONFIG_ANALOG_SENSORS)
	cons_printf("\t'a' to calibrate analogs sensors\n");
#endif
	cons_printf("\t'o' to calibrate odometry\n");
	cons_printf("\t'p' to calibrate hbridge & PWM ctrl\n");
#if defined(CONFIG_SD21)
	cons_printf("\t's' to calibrate servos (sd21 card)\n");
#endif
	cons_printf("\t'r' to calibrate controller\n");
	cons_printf("\t'g' to calibrate game planner\n");
	cons_printf("\n");
	cons_printf("\t'h' to display this help\n");
	cons_printf("\t'e' to exit calibration mode\n");
	cons_printf("\n");
}

static void *task_calibration_entry(void *arg)
{
//	int16_t autoboot_ms = 3000;
	int c;
	uint8_t quit = 0;

	(void)arg;

//	/* wait for keypress, or schedule */
//	while (!usart_is_data_arrived(&USART_CONSOLE)) {
//
//		cons_printf("Press a key to enter calibration... %ds remaining\r",
//			autoboot_ms / 1000);
//
//		kos_set_next_schedule_delay_ms(250);
//		autoboot_ms -= 250;
//
//		if (autoboot_ms > 0)
//			kos_yield();
//		else
//			/* time elapsed, we bypass calibration
//			 * and continue to game mode. */
//			goto exit_point;
//	}
	cons_printf("\n\n");
	getchar();

//	mcurses_init();
	mach_calibration_usage();

	while (!quit) {

		/* display prompt */
		cons_printf("$ ");

		/* wait for command */
		c = cons_getchar();
		cons_printf("%c\n", c);

		switch (c) {
#if defined(CONFIG_ANALOG_SENSORS)
		case 'a':
			analog_sensor_enter_calibration(&ana_sensors);
			break;
#endif
		case 'o':
			//encoder_enter_calibration();
			/* TODO; odometry_enter_calibration */
			break;
		case 'p':
			//hbridge_enter_calibration(&hbridges);
			break;
#if defined(CONFIG_SD21)
		case 's':
			sd21_enter_calibration(&sd21);
			break;
#endif
		case 'r':
			controller_enter_calibration();
			break;
		case 'g':
			planner_enter_calibration();
			break;
		case 'h':
			mach_calibration_usage();
			break;
		case 'e':
			quit = 1;
			break;
		default:
			cons_printf("\n");
			break;
		}
	}

//exit_point:
	cons_printf("calibration ended\n");
	planner_start_game();
	kos_task_exit();

	return 0;
}
#endif /* CONFIG_CALIBRATION */

/*
 * Tasks registration
 */

//#define TASK_CALIB_STACK	256
//#define TASK_CTRL_STACK		512
//#define TASK_PLAN_STACK		512

#if defined(CONFIG_CALIBRATION)
char calib_thread_stack[THREAD_STACKSIZE_DEFAULT];
#endif
char controller_thread_stack[THREAD_STACKSIZE_DEFAULT];
char planner_thread_stack[THREAD_STACKSIZE_DEFAULT];

/* Note: last task registered will be the first to be scheduled.
 *    running order can be important in current tasks design */
void mach_tasks_init(void)
{
//	kos_init();
//
#if defined(CONFIG_CALIBRATION)
	//kos_new_task(task_calibration_entry, "CALIB", TASK_CALIB_STACK);
    thread_create(calib_thread_stack, sizeof(calib_thread_stack),
                  0, 0,
                  task_calibration_entry, NULL, "calibration");
#else
	planner_start_game();
#endif
	//kos_new_task(task_controller_update, "CTRL", TASK_CTRL_STACK);
	//kos_new_task(task_planner, "PLAN", TASK_PLAN_STACK);
    thread_create(controller_thread_stack, sizeof(controller_thread_stack),
                  0, 0,
                  task_controller_update, NULL, "motion_ctrl");

    thread_create(planner_thread_stack, sizeof(planner_thread_stack),
                  0, 0,
                  task_planner, NULL, "game_planner");

}
