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

    if (argc != 4) {
        std::cerr << "Bad argument number, usage: " << argv[0] << " <motor_driver_id> <motor_id> <signed_pwm_percents>" << std::endl;
        return -1;
    }

    motor_driver_t motor_driver = atoi(argv[1]);
    if(motor_driver >= MOTOR_DRIVER_NUMOF) {
        std::cerr << "Bad motor driver ID, should be strictly lesser than " << MOTOR_DRIVER_NUMOF << std::endl;
        return -1;
    }

    uint8_t motor = (uint8_t)atoi(argv[2]);
    if(motor >= motor_driver_config[motor_driver].nb_motors) {
        std::cerr << "Bad motor ID, should be strictly lesser than " << motor_driver_config[motor_driver].nb_motors << std::endl;
        return -1;
    }

    int32_t pwm_percent = (int16_t)atoi(argv[3]);
    if (abs(pwm_percent) > 100) {
        std::cerr << "Bad pwm value, should be a signed percentage"  << std::endl;
        return -1;
    }

    cogip::pf::motion_control::pf_disable_motion_control();

    int16_t pwm_resolution = (int16_t)((motor_driver_config[motor_driver].pwm_resolution * pwm_percent) / 100);

    int32_t qdec_value = 0;
    int timeout = 3000;

    std::cout << "### Testing motor " << (int)motor << " of motor driver " << motor_driver << std::endl;

    /* Reset qdec */
    if (motor < QDEC_NUMOF)
        qdec_read_and_reset(QDEC_DEV(motor));

    motor_enable(motor_driver, motor);

    /* Forward */
    int chrono = timeout;
    std::cout << "    Forward move" << std::endl;
    motor_set(motor_driver, motor, pwm_resolution);
    while (chrono--) {
        if (motor < QDEC_NUMOF)
            qdec_value += qdec_read_and_reset(QDEC_DEV(motor));
        ztimer_sleep(ZTIMER_MSEC, 1);
    }
    std::cout << "    qdec value = " << qdec_value;
    std::cout << "    Done" << std::endl;

    /* Stop */
    qdec_value = 0;
    std::cout << "    Stop" << std::endl;
    motor_brake(motor_driver, motor);
    ztimer_sleep(ZTIMER_SEC, 3);
    std::cout << "    Done" << std::endl;

    /* Backward */
    chrono = timeout;
    std::cout << "    Backward move" << std::endl;
    motor_set(motor_driver, motor, -pwm_resolution);
    while (chrono--) {
        if (motor < QDEC_NUMOF)
            qdec_value += qdec_read_and_reset(QDEC_DEV(motor));
        ztimer_sleep(ZTIMER_MSEC, 1);
    }
    std::cout << "    qdec value = " << qdec_value;
    std::cout << "    Done" << std::endl;

    /* Stop */
    std::cout << "    Stop" << std::endl;
    motor_brake(motor_driver, motor);
    std::cout << "    Done" << std::endl;

    ztimer_sleep(ZTIMER_SEC, 2);

    motor_disable(motor_driver, motor);

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
