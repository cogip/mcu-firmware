/*
 * Copyright (C) 2023 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-board COGIP 2018 Cortex board
 * @ingroup     boards_cogip-board
 * @brief       Support for the COGIP 2018 Cortex board
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

#include <motor_driver.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Not connected GPIO for fake use */
#define GPIO_OUTPUT_UNUSED     GPIO_PIN(PORT_B, 15)

/* LEDs */
#define HEARTBEAT_LED   GPIO_PIN(PORT_A, 0)

/* Vacuum pumps */
#define GPIO_VACUUM1_ENABLE GPIO_PIN(PORT_C, 13)
#define GPIO_VACUUM1_TEST GPIO_PIN(PORT_C, 0)
#define GPIO_VACUUM2_ENABLE GPIO_PIN(PORT_C, 14)
#define GPIO_VACUUM2_TEST GPIO_PIN(PORT_C, 1)
#define GPIO_VACUUM3_ENABLE GPIO_PIN(PORT_C, 15)
#define GPIO_VACUUM3_TEST GPIO_PIN(PORT_C, 2)

/* Servomotors */
#define LX_DIR_PIN      GPIO_PIN(PORT_B, 3)
#define LX_UART_DEV     2

/* Motors */
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

/* Quadrature decoding */
#define QDEC_MODE           QDEC_X4
#define QDEC_LEFT_POLARITY  1
#define QDEC_RIGHT_POLARITY -1

/**
 * @brief Describe DC motor with PWM channel and GPIOs
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .pwm_dev = 0,
        .mode = MOTOR_DRIVER_1_DIR_BRAKE,
        .mode_brake = MOTOR_BRAKE_HIGH,
        .pwm_mode = PWM_LEFT,
        .pwm_frequency = 100000U,
        .pwm_resolution = 450U,
        .nb_motors = 2,
        .motors = {
            /* Left motor */
            {
                .pwm_channel = 0,
                .gpio_enable = GPIO_PIN(PORT_B, 10),
                .gpio_dir0 = GPIO_PIN(PORT_B, 2),
                .gpio_dir1_or_brake = GPIO_PIN(PORT_B, 12),
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 1,
            },
            /* Right motor */
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_PIN(PORT_B, 10),
                .gpio_dir0 = GPIO_PIN(PORT_B, 0),
                .gpio_dir1_or_brake = GPIO_PIN(PORT_C, 4),
                .gpio_dir_reverse = 1,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 1,
            }
        },
        .cb = NULL
    },
    {
        .pwm_dev = 1,
        .mode = MOTOR_DRIVER_1_DIR,
        .mode_brake = MOTOR_BRAKE_LOW,
        .pwm_mode = PWM_LEFT,
        .pwm_frequency = 20000U,
        .pwm_resolution = 100U,
        .nb_motors = 2,
        .motors = {
            /* Lift motor */
            {
                .pwm_channel = 2,
                .gpio_enable = GPIO_PIN(PORT_A, 8),
                .gpio_dir0 = GPIO_PIN(PORT_A, 11),
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
            /* Ball launcher (no direction, no enable, just PWM) & conveyor (no PWM, no direction, just the enable) */
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_PIN(PORT_B, 1),
                .gpio_dir0 = GPIO_OUTPUT_UNUSED,
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
        },
        .cb = NULL
    }
};

#define MOTOR_DRIVER_NUMOF      (sizeof(motor_driver_config) / sizeof(motor_driver_config[0]))

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

/** @} */
