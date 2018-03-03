#include <stdio.h>
#include "xtimer.h"
#include "timex.h"
#include "thread.h"
#ifdef MOTOR_DRIVER_NUMOF
#include "periph/qdec.h"
#include "motor_driver.h"
#endif
#include "platform.h"
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
    uint32_t freq = 20000U;
    uint16_t res = 2000U;
#endif
    xtimer_ticks32_t last_wakeup/*, start*/;

    (void) arg;

    last_wakeup = xtimer_now();
    //start = xtimer_now();

#ifndef MOTOR_DRIVER_NUMOF
    for (;;)
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
#else
    motor_t left_motor;
    motor_t right_motor;

    motor_pwm_init(0, freq, res);

    motor_init(&left_motor, 0, 0, 0, 1);
    motor_init(&right_motor, 0, 1, 2, 3);

    while (1) {
        //printf("MOTION CONTROL  : slept until %" PRIu32 "\n", xtimer_usec_from_ticks(last_wakeup) - xtimer_usec_from_ticks(start));
        if (count < 500)
        {
            motor_set(&left_motor, 1, duty_cycle);
            motor_set(&right_motor, 0, duty_cycle++);
            count++;
        }
        else if (duty_cycle == 0) {
            count = 0;
        }
        else
        {
            motor_set(&left_motor, 1, duty_cycle);
            motor_set(&right_motor, 0, duty_cycle--);
        }
        //printf("QDEC_VALUE      : %d\n", qdec_read(0));
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
//    thread_create(motion_control_thread_stack, sizeof(motion_control_thread_stack),
//                  0, 0,
//                  motion_control_thread, NULL, "motion_ctrl");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
