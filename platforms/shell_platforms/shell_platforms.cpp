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
#include "platform.hpp"
#include "ctrl.hpp"
#include "shell_menu/shell_menu.hpp"
#include "shell_platforms.hpp"
#include "utils.hpp"

/* Enable or disable debug for this file only */
#define ENABLE_DEBUG        (0)
#include "debug.h"

/* Controller */
static ctrl_t *ctrl = NULL;

static int _cmd_print_state_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    pf_print_state();

    return EXIT_SUCCESS;
}

static int _cmd_motors_test_cb(int argc, char **argv)
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

static cogip::shell::Command _cmd_print_state = { "_state", "Print current state", _cmd_print_state_cb };

static cogip::shell::Menu _menu_pf = { "Platforms menu", "pf_menu", &cogip::shell::root_menu() };
static cogip::shell::Command _cmd_motors_test = { "mt", "Test all DC motors", _cmd_motors_test_cb };

void pf_shell_init(void)
{
    ctrl = pf_get_ctrl();

    /* Global commands */
    cogip::shell::add_global_command(&_cmd_print_state);

    /* Platforms commands */
    _menu_pf.push_back(&_cmd_motors_test);
}
