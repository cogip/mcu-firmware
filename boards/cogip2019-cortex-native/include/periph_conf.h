/*
 * Copyright (C) 2019 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * This file is copied from RIOT-OS native board sources
 */

#pragma once

#include "periph_board.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name hardware timer clock skew avoidance
 * @{
 */
#define NATIVE_TIMER_MIN_RES 200
/** @} */

/**
 * @name Random Number Generator configuration
 * @{
 */
#define RANDOM_NUMOF        (1U)
/** @} */

/**
 * @name Timer peripheral configuration
 * @{
 */
#define TIMER_NUMOF        (1U)
#define TIMER_0_EN         1

/**
 * @brief xtimer configuration
 */
#define XTIMER_OVERHEAD 14

/* timer_set_absolute() has a high margin for possible underflow if set with
 * value not far in the future. To prevent this, we set high backoff values
 * here.
 */
#define XTIMER_BACKOFF      200
#define XTIMER_ISR_BACKOFF  200

/** @} */

/**
 * @brief UART configuration
 * @{
 */
#ifndef UART_NUMOF
#define UART_NUMOF (1U)
#endif
/** @} */

/**
 * @brief PWM configuration
 */
#ifndef PWM_NUMOF
#define PWM_NUMOF (8U)
#endif

/**
 * @brief QDEC configuration
 */
#ifndef QDEC_NUMOF
#define QDEC_NUMOF (8U)
#endif

/**
 * @brief I2C configuration
 * @{
 */
#ifndef I2C_NUMOF
#define I2C_NUMOF (8U)
#endif

static const i2c_conf_t i2c_config[I2C_NUMOF];
/** @} */


#ifdef __cplusplus
}
#endif

/** @} */
