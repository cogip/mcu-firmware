#include <stdio.h>
#include "xtimer.h"
#include "timex.h"
#include "thread.h"
#ifdef MOTOR_DRIVER_NUMOF
#include "periph/qdec.h"
#include "motor_driver.h"
#endif
#include "platform.h"
#include "platform_task.h"
#include "shell.h"
#include "shell_commands.h"

/* set interval to 20 milli-second */
#define INTERVAL (20U * US_PER_MS)

char motion_control_thread_stack[THREAD_STACKSIZE_DEFAULT];

void *motion_control_thread(void *arg)
{
#ifdef MOTOR_DRIVER_NUMOF
	uint16_t duty_cycle = 0;
	uint16_t count = 0;
	uint8_t dir = 0;
#endif
	xtimer_ticks32_t last_wakeup/*, start*/;

	(void) arg;

	last_wakeup = xtimer_now();
	//start = xtimer_now();

#ifndef MOTOR_DRIVER_NUMOF
	for (;;)
		xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
#else

	while (motor_driver_init(0))
	{
		puts("ERROR Initializing motor driver !!!");
		xtimer_periodic_wakeup(&last_wakeup, INTERVAL*10);
	}

	while (1) {
		//printf("MOTION CONTROL  : slept until %" PRIu32 "\n", xtimer_usec_from_ticks(last_wakeup) - xtimer_usec_from_ticks(start));
		if (count < 500)
		{
			if (motor_set(0, HBRIDGE_MOTOR_LEFT, dir, duty_cycle))
				printf("Cannot set PWM duty cycle for motor %u\n", HBRIDGE_MOTOR_LEFT);
			if (motor_set(0, HBRIDGE_MOTOR_RIGHT, dir, duty_cycle++))
				printf("Cannot set PWM duty cycle for motor %u\n", HBRIDGE_MOTOR_RIGHT);
			count++;
		}
		else if (duty_cycle == 0) {
			count = 0;
			dir = dir ? 0 : 1;
		}
		else
		{
			if (motor_set(0, HBRIDGE_MOTOR_LEFT, dir, duty_cycle))
				printf("Cannot set PWM duty cycle for motor %u\n", HBRIDGE_MOTOR_LEFT);
			if (motor_set(0, HBRIDGE_MOTOR_RIGHT, dir, duty_cycle--))
				printf("Cannot set PWM duty cycle for motor %u\n", HBRIDGE_MOTOR_RIGHT);
		}
		//printf("QDEC_VALUE	  : %d\n", qdec_read(0));
		xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
	}
#endif /* MOTOR_DRIVER_NUMOF */
	return NULL;
}

int main(void)
{
	mach_setup();

	mach_tasks_init();
//	mach_sched_init();

//	mach_sched_run();
//	thread_create(motion_control_thread_stack, sizeof(motion_control_thread_stack),
//				  0, 0,
//				  motion_control_thread, NULL, "motion_ctrl");

	/*char line_buf[SHELL_DEFAULT_BUFSIZE];
	shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);*/

	return 0;
}
