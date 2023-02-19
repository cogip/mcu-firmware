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
#include "motion_control.hpp"
#include "platform.hpp"
#include "shell_menu/shell_menu.hpp"
#include "shell_platforms.hpp"
#include "utils.hpp"

static int _cmd_motors_test_cb(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int nb_motors = 0;

    for (motor_driver_t i = 0; i < MOTOR_DRIVER_NUMOF; i++) {
        int nb_motors_tmp = motor_driver_config[i].nb_motors;
        int pwm_resolution = motor_driver_config[i].pwm_resolution;

        for (int j = 0; j < nb_motors_tmp; j++) {

            int32_t qdec_value = 0;
            int timeout = 3000;

            std::cout << "### Testing motor " << j << " of motor driver " << i << std::endl;

            /* Reset qdec */
            qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            /* Forward */
            std::cout << "    Forward move" << std::endl;
            motor_set(i, j, pwm_resolution);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                ztimer_sleep(ZTIMER_MSEC, 1);
            }
            std::cout << "    qdec value = " << qdec_value;
            std::cout << "    Done" << std::endl;

            /* Stop */
            timeout = 3000;
            qdec_value = 0;
            std::cout << "    Stop" << std::endl;
            motor_brake(i, j);
            ztimer_sleep(ZTIMER_SEC, 3);
            std::cout << "    Done" << std::endl;

            /* Backward */
            std::cout << "    Backward move" << std::endl;
            motor_set(i, j, -pwm_resolution);
            while (timeout--) {
                qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                ztimer_sleep(ZTIMER_MSEC, 1);
            }
            std::cout << "    qdec value = " << qdec_value;
            std::cout << "    Done" << std::endl;

            /* Stop */
            std::cout << "    Stop" << std::endl;
            motor_brake(i, j);
            std::cout << "    Done" << std::endl;

            ztimer_sleep(ZTIMER_SEC, 2);
        }

        nb_motors += nb_motors_tmp;
    }

    return EXIT_SUCCESS;
}

static cogip::shell::Menu _menu_pf = { "Platforms menu", "pf_menu", &cogip::shell::root_menu() };
static cogip::shell::Command _cmd_motors_test = { "mt", "Test all DC motors", _cmd_motors_test_cb };

void pf_shell_init(void)
{
    /* Platforms commands */
    _menu_pf.push_back(&_cmd_motors_test);
}
