// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     drivers_soft_i2c
/// @{
/// @file
/// @brief       Default software I2C configuration
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#ifndef SOFT_I2C_PARAMS_H
#define SOFT_I2C_PARAMS_H

#include "board.h"
#include "soft_i2c/soft_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SOFT_I2C_PARAM_SCL
#define SOFT_I2C_PARAM_SCL GPIO_PIN(PORT_A, 6)
#endif

#ifndef SOFT_I2C_PARAM_SDA
#define SOFT_I2C_PARAM_SDA GPIO_PIN(PORT_A, 4)
#endif

#ifndef SOFT_I2C_PARAM_SPEED
#define SOFT_I2C_PARAM_SPEED (100000UL)
#endif

#ifndef SOFT_I2C_PARAMS
#define SOFT_I2C_PARAMS                                                                            \
    {                                                                                              \
        .scl_pin = SOFT_I2C_PARAM_SCL, .sda_pin = SOFT_I2C_PARAM_SDA,                              \
        .speed_hz = SOFT_I2C_PARAM_SPEED,                                                          \
    }
#endif

static const soft_i2c_conf_t soft_i2c_config[] = {
    SOFT_I2C_PARAMS,
};

#define SOFT_I2C_NUMOF ARRAY_SIZE(soft_i2c_config)

#ifdef __cplusplus
}
#endif

#endif /* SOFT_I2C_PARAMS_H */

/// @}
