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

#ifndef PCA9685_PARAMS_H
#define PCA9685_PARAMS_H

#include "board.h"
#include "pca9685.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
/** device is I2C_DEV(0) */
#define PCA9685_PARAM_DEV           I2C_DEV(0)

/** device address is PCA9685_I2C_ADDR */
#define PCA9685_PARAM_ADDR          (PCA9685_I2C_BASE_ADDR + 0)

/** Invert outputs: no */
#define PCA9685_PARAM_INV           (false)

/** PWM mode for all channels: PWM_LEFT */
#define PCA9685_PARAM_MODE          (PWM_LEFT)

/** PWM frequency in Hz */
#define PCA9685_PARAM_FREQ          (50)

/** PWM resolution */
#define PCA9685_PARAM_RES           (2000)

/** Output enable pin: not used */
#define PCA9685_PARAM_OE_PIN        (GPIO_UNDEF)

/** EXTCLK frequency and pin: not used */
#define PCA9685_PARAM_EXT_FREQ      (0)

/** Output driver mode: totem pole */
#define PCA9685_PARAM_OUT_DRV       (PCA9685_TOTEM_POLE)

/** Output driver mode: totem pole */
#define PCA9685_PARAM_OUT_NE        (PCA9685_OFF)

/**
 * @brief   Allocate some memory to store the actual configuration
 */
static const pca9685_params_t pca9685_params = {
    .i2c_dev = PCA9685_PARAM_DEV,
    .i2c_addr = PCA9685_PARAM_ADDR,
    .mode = PCA9685_PARAM_MODE,
    .freq = PCA9685_PARAM_FREQ,
    .res = PCA9685_PARAM_RES,
    .inv = PCA9685_PARAM_INV,
    .ext_freq = PCA9685_PARAM_EXT_FREQ,
    .oe_pin = PCA9685_PARAM_OE_PIN,
    .out_drv = PCA9685_PARAM_OUT_DRV,
    .out_ne = PCA9685_PARAM_OUT_NE,
};

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_PARAMS_H */
/** @} */
