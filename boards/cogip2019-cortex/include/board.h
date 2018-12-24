/*
 * Copyright (C) 2018 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip2018-cortex COGIP 2018 Cortex board
 * @ingroup     boards_cogip2018-cortex
 * @brief       Support for the COGIP 2018 Cortex board
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Describe DC motor with PWM channel and GPIOs
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .mode = MOTOR_DRIVER_1_DIR,
        .pwm_dev = 0,
        .pwm_frequency = 20000U,
        .pwm_resolution = 2000U,
        .nb_motors = 2,
        .motors = {
            {
                .pwm_channel = 0,
                .gpio_enable = GPIO_UNDEF,
                .gpio_dir0 = GPIO_PIN(PORT_A, 4),
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_UNDEF,
                .gpio_dir0 = GPIO_PIN(PORT_B, 8),
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 1,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
        },
    },
    {
        .mode = MOTOR_DRIVER_1_DIR,
        .pwm_dev = 1,
        .pwm_frequency = 30U,
        .pwm_resolution = 10U,
        .nb_motors = 1,
        .motors = {
            {
                .pwm_channel = 3,
                .gpio_enable = GPIO_PIN(PORT_C, 11),
                .gpio_dir0 = GPIO_PIN(PORT_A, 10),
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 1,
                .gpio_brake_invert = 0,
            },
        },
    },
};

#define MOTOR_DRIVER_NUMOF      (sizeof(motor_driver_config) / sizeof(motor_driver_config[0]))

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
