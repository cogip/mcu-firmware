/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip-board-ng
 * @brief       Default COGIP configuration for Texas Instruments PCF857X I2C I/O expanders
 * @{
 *
 * @file
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef PCF857X_PARAMS_H
#define PCF857X_PARAMS_H

#include "pcf857x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
/** device is I2C_DEV(0) */
#define PCF857X_PARAM_DEV       I2C_DEV(0)

/** I2C slave address offset is 0 */
#define PCF857X_PARAM_ADDR      (0)

/** PCF857X expander variant used depends on enabled pseudomodules*/
#define PCF857X_PARAM_EXP       (PCF857X_EXP_PCF8575)

/** MCU interrupt pin */
#define PCF857X_PARAM_INT_PIN   (GPIO_PIN(PORT_B, 5))

/** Default configuration parameter set */
#define PCF857X_PARAMS { \
        .dev = PCF857X_PARAM_DEV, \
        .addr = PCF857X_PARAM_ADDR, \
        .exp = PCF857X_PARAM_EXP, \
        .int_pin = PCF857X_PARAM_INT_PIN, \
}

/**
 * @brief   Allocate some memory to store the actual configuration
 */
static const pcf857x_params_t pcf857x_params =
    PCF857X_PARAMS;

#ifdef __cplusplus
}
#endif

#endif /* PCF857X_PARAMS_H */
/** @} */
