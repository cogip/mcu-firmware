/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    driver_vacuum_pump COGIP vacuum pump driver
 * @ingroup     drivers
 * @brief       COGIP vacuum pump driver
 *
 * The COGIP vacuum pump board is driven by a GPIO to enable/disable the pump.
 * The board also has a vacuum sensor that can be connected to an other test
 * GPIO.
 *  - start/stop GPIO (output): 1 for start, 0 for stop
 *  - test GPIO (input): 1 for under vacuum pressure, 0 for no pressure
 *
 * @{
 * @file
 * @brief       Public API for COGIP vacuum pump driver
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdint.h>
#include <stdbool.h>

/* RIOT includes */
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   COGIP vacuum pump parameters
 */
typedef struct {
    gpio_t gpio_enable;         /**< start/stop pump */
    gpio_t gpio_test;           /**< vacuum sensor */
} vacuum_pump_params_t;

/**
 * @brief   Default COGIP vacuum pump identifier definition
 */
typedef unsigned int vacuum_pump_t;

/**
 * @brief Initialize COGIP vacuum pump device
 *
 * @param[in]   vacuum_pump vacuum_pump device id to initialize
 * @param[in]   params      vacuum_pump parameters
 *
 * @return      0 on success, not 0 otherwise
 */
int vacuum_pump_init(const vacuum_pump_t vacuum_pump, const vacuum_pump_params_t *params);

/**
 * @brief Start COGIP vacuum pump device
 *
 * @param[in]   vacuum_pump vacuum_pump device id
 */
void vacuum_pump_start(const vacuum_pump_t vacuum_pump);

/**
 * @brief Stop COGIP vacuum pump device
 *
 * @param[in]   vacuum_pump vacuum_pump device id
 */
void vacuum_pump_stop(const vacuum_pump_t vacuum_pump);

/**
 * @brief Get vacuum sensor state
 *
 * @param[in]   vacuum_pump      vacuum_pump device id
 *
 * @return      vacuum sensor state, 1 under pressure, 0 otherwise
 */
bool vacuum_pump_is_under_pressure(const vacuum_pump_t vacuum_pump);

#ifdef __cplusplus
}
#endif

/** @} */
