/*
 * Copyright (C) 2021 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-nucleo-f446re COGIP test board
 * @ingroup     boards
 * @brief       Support for COGIP test board, inherited from nucleo-f446re
 * @{
 *
 * @file
 * @brief       Pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

#include "board_nucleo.h"
#include "arduino_pinmap.h"
#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Motors */
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

/* Quadrature decoding */
#define QDEC_MODE           QDEC_X4
#define QDEC_LEFT_POLARITY  -1
#define QDEC_RIGHT_POLARITY -1

/**
 * @brief Describe DC motor with PWM channel and GPIOs
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .pwm_dev = 1,
        .mode = MOTOR_DRIVER_2_DIRS,
        .mode_brake = MOTOR_BRAKE_LOW,
        .pwm_mode = PWM_LEFT,
        .pwm_frequency = 20000U,
        .pwm_resolution = 1500U,
        .nb_motors = 1,
        .motors = {
            /* Left motor */
            {
                .pwm_channel = 2,
                .gpio_enable = GPIO_PIN(PORT_A, 12),
                .gpio_dir0 = GPIO_PIN(PORT_B, 1),
                .gpio_dir1_or_brake = GPIO_PIN(PORT_B, 2),
                .gpio_dir_reverse = 1,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
        },
        .cb = NULL
    },
    {
        .pwm_dev = 1,
        .mode = MOTOR_DRIVER_2_DIRS,
        .mode_brake = MOTOR_BRAKE_LOW,
        .pwm_mode = PWM_LEFT,
        .pwm_frequency = 20000U,
        .pwm_resolution = 1500U,
        .nb_motors = 1,
        .motors = {
            /* Right motor */
            {
                .pwm_channel = 0,
                .gpio_enable = GPIO_PIN(PORT_A, 11),
                .gpio_dir0 = GPIO_PIN(PORT_B, 14),
                .gpio_dir1_or_brake = GPIO_PIN(PORT_B, 15),
                .gpio_dir_reverse = 1,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
        },
        .cb = NULL
    },
};

#define MOTOR_DRIVER_NUMOF      ARRAY_SIZE(motor_driver_config)
/** @} */

#ifdef __cplusplus
}
#endif
