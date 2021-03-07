/* System includes */
#include <inttypes.h>
#include <thread.h>
#include <stdlib.h>
#include <string.h>

/* RIOT includes */
#define ENABLE_DEBUG        (0)
#include "debug.h"
#include "log.h"
#include "shell.h"
#include "xtimer.h"

/* Project includes */
#include "avoidance.h"
#include "ctrl.h"
#include "ctrl.h"
#include "shell_menu.h"
#include "shell_platforms.h"


#define GLOBAL_MENU "_global"

/* Controller */
static ctrl_t* ctrl = NULL;

int pf_print_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf(
        "{"
          "\"mode\": \"%u\", "
          "\"pose_current\": "
          "{"
            "\"O\": \"%lf\", "
            "\"x\": \"%lf\", "
            "\"y\": \"%lf\""
          "}, "
          "\"pose_order\": "
          "{"
            "\"O\": \"%lf\", "
            "\"x\": \"%lf\", "
            "\"y\": \"%lf\""
          "}, "
          "\"cycle\": \"%"PRIu32"\", "
          "\"speed_current\": "
          "{"
            "\"distance\": \"%lf\", "
            "\"angle\": \"%lf\""
          "}, "
          "\"speed_order\": "
          "{"
            "\"distance\": \"%lf\", "
            "\"angle\": \"%lf\""
          "}"
        "}\n",
        ctrl->control.current_mode,
        ctrl->control.pose_current.O,
        ctrl->control.pose_current.x,
        ctrl->control.pose_current.y,
        ctrl->control.pose_order.O,
        ctrl->control.pose_order.x,
        ctrl->control.pose_order.y,
        ctrl->control.current_cycle,
        ctrl->control.speed_current.distance,
        ctrl->control.speed_current.angle,
        ctrl->control.speed_order.distance,
        ctrl->control.speed_order.angle
    );

    return EXIT_SUCCESS;
}

int pf_motors_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int nb_motors = 0;

    ctrl_set_mode(ctrl, CTRL_MODE_IDLE);

    for (motor_driver_t i = 0; i < MOTOR_DRIVER_NUMOF; i++) {
        int nb_motors_tmp = motor_driver_config[i].nb_motors;
        int pwm_resolution = motor_driver_config[i].pwm_resolution;

        for (int j = 0; j < nb_motors_tmp; j++) {

            int32_t qdec_value = 0;
            int timeout = 3000;

            printf("### Testing motor %d of motor driver %d\n", j, i);

            /* Reset qdec */
            qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            /* Forward */
            puts("    Forward move");
            motor_set(i, j, pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            printf("    qdec value = %"PRId32"\n", qdec_value);
            puts("    Done");

            /* Stop */
            timeout = 3000;
            qdec_value = 0;
            puts("    Stop");
            motor_set(i, j, 0);
            xtimer_usleep(3 * US_PER_SEC);
            puts("    Done");

            /* Backward */
            puts("    Backward move");
            motor_set(i, j, -pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            printf("    qdec value = %"PRId32"\n", qdec_value);
            puts("    Done");

            /* Stop */
            puts("    Stop");
            motor_set(i, j, 0);
            puts("    Done");
        }

        nb_motors += nb_motors_tmp;
    }

    ctrl_set_mode(ctrl, CTRL_MODE_STOP);

    return EXIT_SUCCESS;
}

void pf_shell_init(void)
{
    ctrl = pf_get_ctrl();

    /* Global commands */
    static const shell_command_t global_commands[] = {
        { "_state", "Print current state", pf_print_state },
        { "_dyn_obstacles", "Print dynamic obstacles",
            avoidance_print_dyn_obstacles },
        MENU_NULL_CMD
    };
    menu_set_global_commands(global_commands);

    /* Platforms menu and commands */
    shell_menu_t menu = menu_init("Platforms menu", "pf_menu", menu_root);

    const shell_command_t shell_platforms_menu_commands[] = {
        { "mt", "Test all DC motors", pf_motors_test },
        MENU_NULL_CMD,
    };

    menu_add_list(menu, shell_platforms_menu_commands);
}
