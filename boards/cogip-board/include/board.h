/*
 * Copyright (C) 2018 COGIP Robotics association
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

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9548_SENSORS 0

/* Camp selection */
#define GPIO_CAMP       GPIO_PIN(PORT_B, 1)
/* Starting switch */
#define GPIO_STARTER    GPIO_PIN(PORT_B, 2)

/* Pumps */
#define GPIO_BL_PUMP_1  GPIO_PIN(PORT_C, 11)
#define GPIO_BC_PUMP_2  GPIO_PIN(PORT_A, 12)
#define GPIO_BR_PUMP_3  GPIO_PIN(PORT_A, 11)
#define GPIO_FL_PUMP_4  GPIO_PIN(PORT_A, 10)
#define GPIO_FC_PUMP_5  GPIO_PIN(PORT_A, 9)
#define GPIO_FR_PUMP_6  GPIO_PIN(PORT_B, 15)

#define GPIO_DEBUG_LED  GPIO_PIN(PORT_C, 8)

/* Motors */
#define HBRIDGE_MOTOR_LEFT  0
#define HBRIDGE_MOTOR_RIGHT 1

/* Quadrature decoding */
#define QDEC_MODE           QDEC_X4
#define QDEC_LEFT_POLARITY  -1
#define QDEC_RIGHT_POLARITY 1

/**
 * @brief Describe DC motor with PWM channel and GPIOs
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .pwm_dev = 0,
        .mode = MOTOR_DRIVER_1_DIR,
        .mode_brake = MOTOR_BRAKE_LOW,
        .pwm_mode = PWM_LEFT,
        .pwm_frequency = 20000U,
        .pwm_resolution = 1500U,
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
        .cb = NULL
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

/** @} */
