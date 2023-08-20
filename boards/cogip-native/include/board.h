/*
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_native
 *
 * The native board uses call level hardware simulation
 *
 * @{
 *
 * @file
 * @brief       Basic definitions for the native board
 *
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 */

#pragma once

#include <stdint.h>

/* RIOT includes */
#include <motor_driver.h>

/* Project includes */
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fake GPIOs */
enum {
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    NATIVE_GPIO_PORT_NUMOF
};

/* Heartbeat LED */
#define HEARTBEAT_LED   GPIO_PIN(1, 3)

/* GPIOs expander */
#define PCF857X_PORT_0  0


/* Motors */
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

/* Quadrature decoding */
#define QDEC_MODE           QDEC_X1
#define QDEC_LEFT_POLARITY  1
#define QDEC_RIGHT_POLARITY 1

/**
 * @brief Simulate QDEC on motor_set() calls
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 * @param[in] pwm_duty_cycle    Signed PWM duty_cycle to set motor speed and direction
 *
 * @return                      0 on success
 */
void cogip_native_motor_driver_qdec_simulation( \
    const motor_driver_t motor_driver, uint8_t motor_id, \
    int32_t pwm_duty_cycle);

/**
 * @name Describe DC motors with PWM channel and GPIOs
 * @{
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .pwm_dev = 0,
        .mode = MOTOR_DRIVER_1_DIR_BRAKE,
        .mode_brake = MOTOR_BRAKE_LOW,
        .pwm_mode = PWM_LEFT,
        .pwm_frequency = 20000U,
        .pwm_resolution = 1000U,
        .nb_motors = 2,
        .motors = {
            {
                .pwm_channel = 0,
                .gpio_enable = GPIO_PIN(0, 0),
                .gpio_dir0 = GPIO_PIN(0, 0),
                .gpio_dir1_or_brake = GPIO_PIN(0, 0),
                .gpio_dir_reverse = 1,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_PIN(0, 0),
                .gpio_dir0 = GPIO_PIN(0, 0),
                .gpio_dir1_or_brake = GPIO_PIN(0, 0),
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
        },
        .cb = cogip_native_motor_driver_qdec_simulation,
    },
};

#define MOTOR_DRIVER_NUMOF           ARRAY_SIZE(motor_driver_config)
/** @} */

/**
 * @name    ztimer configuration
 * @{
 */
#define CONFIG_ZTIMER_USEC_TYPE    ZTIMER_TYPE_PERIPH_TIMER
#define CONFIG_ZTIMER_USEC_DEV     TIMER_DEV(0)
/* on native, anything can happen... */
#define CONFIG_ZTIMER_USEC_MIN     (64)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
