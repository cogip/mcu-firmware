#include "planner.h"
#include "platform.h"
#include "platform_task.h"
#include "xtimer.h"
#include <thread.h>

#if defined(CONFIG_MOTOR_PAP)
#include "actuators/motor_pap.h"
#endif

static void mach_calibration_usage(void)
{
    cons_printf("\n>>> Entering calibration mode\n\n");

#if defined(CONFIG_ANALOG_SENSORS)
    cons_printf("\t'a' to calibrate analogs sensors\n");
#endif
#if defined(CONFIG_MOTOR_PAP)
    cons_printf("\t'p' to calibrate PAP motor\n");
#endif
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
    ctrl_set_mode(&controller, CTRL_STATE_STOP);

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
			/* TODO; odometry_enter_calibration */
#if defined(CONFIG_MOTOR_PAP)
            case 'p':
                motor_pap_calib();
                break;
#endif
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

    return 0;
}
