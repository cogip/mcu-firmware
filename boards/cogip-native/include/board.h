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
#define HEARTBEAT_LED   GPIO_PIN(PORT_A, 0)

/* GPIOs expander */
#define PCF857X_PORT_0  0

/* Servomotors */
#define LX_DIR_PIN      GPIO_PIN(PORT_B, 3)
#define LX_UART_DEV     2

/* Motion motors simulation */
#define MOTION_MOTORS_POST_CB cogip_native_motor_driver_qdec_simulation

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
    const motor_driver_t *motor_driver, uint8_t motor_id, \
    int32_t pwm_duty_cycle);

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
