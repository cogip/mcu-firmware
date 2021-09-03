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
#include "avoidance.hpp"
#include "obstacles/obstacles.hpp"
#include "platform.hpp"
#include "ctrl.hpp"
#include "shell_menu/shell_menu.hpp"
#include "shell_platforms.hpp"
#include "tracefd/tracefd.hpp"

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
        cogip::shell::rename_command("_trace_off", "_trace_on");
        pf_set_trace_mode(false);
    }
    else {
        cogip::shell::rename_command("_trace_on", "_trace_off");
        pf_set_trace_mode(true);
    }
    return 0;
}

static int _cmd_print_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf_print_state(cogip::tracefd::out);

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

            cogip::tracefd::out.logf("### Testing motor %d of motor driver %d", j, i);

            /* Reset qdec */
            qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            /* Forward */
            cogip::tracefd::out.logf("    Forward move");
            motor_set(i, j, pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            cogip::tracefd::out.logf("    qdec value = %" PRId32, qdec_value);
            cogip::tracefd::out.logf("    Done");

            /* Stop */
            timeout = 3000;
            qdec_value = 0;
            cogip::tracefd::out.logf("    Stop");
            motor_set(i, j, 0);
            xtimer_usleep(3 * US_PER_SEC);
            cogip::tracefd::out.logf("    Done");

            /* Backward */
            cogip::tracefd::out.logf("    Backward move");
            motor_set(i, j, -pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                xtimer_usleep(US_PER_MS);
            }
            cogip::tracefd::out.logf("    qdec value = %" PRId32, qdec_value);
            cogip::tracefd::out.logf("    Done");

            /* Stop */
            cogip::tracefd::out.logf("    Stop");
            motor_set(i, j, 0);
            cogip::tracefd::out.logf("    Done");
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

    cogip::tracefd::out.lock();
    avoidance_print_path(cogip::tracefd::out);
    cogip::tracefd::out.printf("\n");
    cogip::tracefd::out.unlock();

    return EXIT_SUCCESS;
}

void pf_shell_init(void)
{
    ctrl = pf_get_ctrl();

    /* Global commands */
    cogip::shell::add_global_command(new cogip::shell::Command(
        "_state", "Print current state", _cmd_print_state
    ));
    cogip::shell::add_global_command(new cogip::shell::Command(
        "_avoidance_path", "Print avoidance current path", _cmd_print_avoidance_path
    ));
    cogip::shell::add_global_command(new cogip::shell::Command(
        "_trace_on", "Activate/deactivate trace", _cmd_trace_on_off
    ));
#ifdef MODULE_SHMEM
    cogip::shell::add_global_command(new cogip::shell::Command(SHMEM_SET_KEY_CMD));
#endif

    /* Platforms menu and commands */
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Platforms menu", "pf_menu", &cogip::shell::root_menu);

    menu->push_back(new cogip::shell::Command(
        "mt", "Test all DC motors", pf_motors_test));
}
