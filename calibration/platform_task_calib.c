#include "planner.h"
#include "platform.h"
#include "platform_task.h"
#include "xtimer.h"
#include <thread.h>

//FIXME:
#define kos_task_exit()

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

void *task_calibration_entry(void *arg)
{
//	int16_t autoboot_ms = 3000;
	int c = 0;
	uint8_t quit = 0;

	(void)arg;
	controller_set_mode(&controller, &controller_modes[CTRL_STATE_STOP]);

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

//	mcurses_init();
	mach_calibration_usage();

	while (!quit) {

		/* display prompt */
		cons_printf("$ ");

		/* wait for command */
		c = getchar();
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
