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

    uint8_t nb_motors = 0;

    for (motor_driver_t i = 1; i < MOTOR_DRIVER_NUMOF; i++) {
        uint8_t nb_motors_tmp = motor_driver_config[i].nb_motors;
        int16_t pwm_resolution = motor_driver_config[i].pwm_resolution;

        for (uint8_t j = 0; j < nb_motors_tmp; j++) {

            int32_t qdec_value = 0;
            int timeout = 1000;

            std::cout << "### Testing motor " << j << " of motor driver " << i << std::endl;

            /* Reset qdec */
            if (nb_motors + j < QDEC_NUMOF)
                qdec_read_and_reset(QDEC_DEV(nb_motors + j));

            motor_enable(i, j);

            /* Forward */
            std::cout << "    Forward move" << std::endl;
            motor_set(i, j, pwm_resolution);
            while (timeout--) {
                if (nb_motors + j < QDEC_NUMOF)
                    qdec_value += qdec_read_and_reset(QDEC_DEV(nb_motors + j));
                ztimer_sleep(ZTIMER_MSEC, 1);
            }
            std::cout << "    qdec value = " << qdec_value;
            std::cout << "    Done" << std::endl;

            /* Stop */
            timeout = 1000;
            qdec_value = 0;
            std::cout << "    Stop" << std::endl;
            motor_brake(i, j);
            ztimer_sleep(ZTIMER_SEC, 3);
            std::cout << "    Done" << std::endl;

            /* Backward */
            std::cout << "    Backward move" << std::endl;
            motor_set(i, j, -pwm_resolution);
            while (timeout--) {
                if (nb_motors + j < QDEC_NUMOF)
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

            motor_disable(i, j);
        }

        nb_motors += nb_motors_tmp;
    }

    cogip::pf::motion_control::pf_enable_motion_control();

    return EXIT_SUCCESS;
}

static cogip::shell::Menu _menu_pf = { "Platforms menu", "pf_menu", &cogip::shell::root_menu() };
static cogip::shell::Command _cmd_motors_test = { "mt", "Test all DC motors", _cmd_motors_test_cb };

void pf_shell_init(void)
{
    /* Platforms commands */
    _menu_pf.push_back(&_cmd_motors_test);
}
