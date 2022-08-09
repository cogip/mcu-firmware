/* System includes */
#include <inttypes.h>
#include <thread.h>
#include <stdlib.h>
#include <string.h>

/* RIOT includes */
#include "log.h"
#include "motor_driver.h"
#include "shell.h"
#include "ztimer.h"
#include "periph/qdec.h"

/* Project includes */
#include "board.h"
#include "avoidance.hpp"
#include "platform.hpp"
#include "ctrl.hpp"
#include "shell_menu/shell_menu.hpp"
#include "shell_platforms.hpp"
#include "utils.hpp"

#ifdef MODULE_SHMEM
#include "shmem.h"
#endif

/* Enable or disable debug for this file only */
#define ENABLE_DEBUG        (0)
#include "debug.h"

/* Controller */
static ctrl_t *ctrl = NULL;

static int _cmd_print_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf_print_state();

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

            COGIP_DEBUG_COUT("### Testing motor " << j << " of motor driver " << i);

            /* Reset qdec */
            qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            /* Forward */
            COGIP_DEBUG_COUT("    Forward move");
            motor_set(i, j, pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                ztimer_sleep(ZTIMER_MSEC, 1);
            }
            COGIP_DEBUG_COUT("    qdec value = " << qdec_value);
            COGIP_DEBUG_COUT("    Done");

            /* Stop */
            timeout = 3000;
            qdec_value = 0;
            COGIP_DEBUG_COUT("    Stop");
            motor_set(i, j, 0);
            ztimer_sleep(ZTIMER_SEC, 3);
            COGIP_DEBUG_COUT("    Done");

            /* Backward */
            COGIP_DEBUG_COUT("    Backward move");
            motor_set(i, j, -pwm_resolution / 4);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                ztimer_sleep(ZTIMER_MSEC, 1);
            }
            COGIP_DEBUG_COUT("    qdec value = " << qdec_value);
            COGIP_DEBUG_COUT("    Done");

            /* Stop */
            COGIP_DEBUG_COUT("    Stop");
            motor_set(i, j, 0);
            COGIP_DEBUG_COUT("    Done");
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

    avoidance_print_path();

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
#ifdef MODULE_SHMEM
    cogip::shell::add_global_command(new cogip::shell::Command(SHMEM_SET_KEY_CMD));
#endif

    /* Platforms menu and commands */
    cogip::shell::Menu *menu = new cogip::shell::Menu(
        "Platforms menu", "pf_menu", &cogip::shell::root_menu);

    menu->push_back(new cogip::shell::Command(
        "mt", "Test all DC motors", pf_motors_test));
}
