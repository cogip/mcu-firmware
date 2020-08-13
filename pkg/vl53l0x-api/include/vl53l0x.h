/*
 * Copyright (C) 2019 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    pkg_vl53l0x-api   VL53L0X ST API
 * @ingroup     pkg
 * @brief       Provides the vl53l0x ST API already patched for RIOT
 * @see         https://github.com/gdoffe/vl53l0x-api
 *
 * This API aims to drive VL53L0X ToF sensor.
 * It actually performs a single ranging measurement.
 * This API wraps call to the official ST VL53L0X API
 *
 * @{
 *
 * @file
 * @brief       vl53l0x-api RIOT interface
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */
#pragma once

/* RIOT includes */
#include "periph/i2c.h"

/* VL53L0X ST API includes */
#include "vl53l0x_api.h"                                                        
#include "vl53l0x_platform.h"


/**
 * @brief   VL53L0X ToF sensor id
 */
typedef uint16_t vl53l0x_t;

/**
 * @brief   VL53L0X ToF sensor configuration
 */
typedef struct {
    i2c_t       i2c_dev;    /**< I2C bus */
    uint16_t    i2c_addr;   /**< I2C ToF address */
} vl53l0x_conf_t;


/**
 * @brief Initialize given VL53L0X ToF sensor
 *
 * param[in]    dev         VL53L0X ToF sensor id
 *
 * @return                  0 on success
 * @return                  not 0 otherwise
 */
int vl53l0x_init_dev(vl53l0x_t dev);

/**
 * @brief Initialize all VL53L0X ToF sensors
 *
 * @return
 */
void vl53l0x_init(void);

/**
 * @brief Reset given VL53L0X ToF sensor
 *
 * param[in]    dev         VL53L0X ToF sensor id
 *
 * @return                  0 on success
 * @return                  not 0 otherwise
 */
int vl53l0x_reset_dev(vl53l0x_t dev);

/**
 * @brief Reset all VL53L0X ToF sensors
 *
 * @return
 */
void vl53l0x_reset(void);

/**
 * @brief Perform a continuous ranging measurement
 *
 * @param[in]   dev         VL53L0X ToF sensor id
 *
 * @return                  distance measure (millimeter)
 */
uint16_t vl53l0x_continuous_ranging_get_measure(vl53l0x_t dev);

/** @} */
