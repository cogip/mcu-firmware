/* System includes */
#include <inttypes.h>
#include <thread.h>
#include <stdlib.h>
#include <string.h>

/* RIOT includes */
#include "log.h"
#include "shell.h"
#include "xtimer.h"
#include "periph/qdec.h"

/* Project includes */
#include "avoidance.h"
#include "obstacles.h"
#include "platform.h"
#include "ctrl.h"
#include "shell_menu.h"
#include "shell_platforms.h"
#include "tracefd.h"

#ifdef MODULE_SHMEM
#include "shmem.h"
#endif

/* Enable or disable debug for this file only */
#define ENABLE_DEBUG        (0)
#include "debug.h"

/* Controller */
static ctrl_t *ctrl = NULL;

static int _cmd_trace_on_off(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (pf_trace_on()) {
        menu_rename_command("_trace_off", "_trace_on");
        pf_set_trace_mode(false);
    }
    else {
        menu_rename_command("_trace_on", "_trace_off");
        pf_set_trace_mode(true);
    }
    return 0;
}

static int _cmd_print_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf_print_state(tracefd_stdout);

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

            tracefd_jlog(tracefd_stdout, "### Testing motor %d of motor driver %d", j, i);

            /* Reset qdec */
            qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            /* Forward */
            tracefd_jlog(tracefd_stdout, "    Forward move");
            motor_set(i, j, pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            tracefd_jlog(tracefd_stdout, "    qdec value = %" PRId32, qdec_value);
            tracefd_jlog(tracefd_stdout, "    Done");

            /* Stop */
            timeout = 3000;
            qdec_value = 0;
            tracefd_jlog(tracefd_stdout, "    Stop");
            motor_set(i, j, 0);
            xtimer_usleep(3 * US_PER_SEC);
            tracefd_jlog(tracefd_stdout, "    Done");

            /* Backward */
            tracefd_jlog(tracefd_stdout, "    Backward move");
            motor_set(i, j, -pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            tracefd_jlog(tracefd_stdout, "    qdec value = %" PRId32, qdec_value);
            tracefd_jlog(tracefd_stdout, "    Done");

            /* Stop */
            tracefd_jlog(tracefd_stdout, "    Stop");
            motor_set(i, j, 0);
            tracefd_jlog(tracefd_stdout, "    Done");
        }

        nb_motors += nb_motors_tmp;
    }

    ctrl_set_mode(ctrl, CTRL_MODE_STOP);

    return EXIT_SUCCESS;
}

int _cmd_print_avoidance_path(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    tracefd_lock(tracefd_stdout);
    avoidance_print_path(tracefd_stdout);
    tracefd_printf(tracefd_stdout, "\n");
    tracefd_unlock(tracefd_stdout);

    return EXIT_SUCCESS;
}

void pf_shell_init(void)
{
    ctrl = pf_get_ctrl();

    /* Global commands */
    static const shell_command_t global_commands[] = {
        { "_state", "Print current state", _cmd_print_state },
        { "_avoidance_path", "Print avoidance current path", _cmd_print_avoidance_path },
        { "_trace_on", "Activate/deactivate trace", _cmd_trace_on_off },
#ifdef MODULE_SHMEM
        SHMEM_SET_KEY_CMD,
#endif
        MENU_NULL_CMD
    };
    menu_set_global_commands(global_commands);

    /* Platforms menu and commands */
    shell_menu_t menu = menu_init("Platforms menu", "pf_menu", menu_root, NULL);

    const shell_command_t shell_platforms_menu_commands[] = {
        { "mt", "Test all DC motors", pf_motors_test },
        MENU_NULL_CMD,
    };

    menu_add_list(menu, shell_platforms_menu_commands);
}
