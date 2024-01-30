/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_vacuum_pump
 * @{
 *
 * @file
 * @brief       Parameters for COGIP vacuum pump driver
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

#include "board.h"
#include "vacuum_pump.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Vacuum pump configuration
 */
static const vacuum_pump_params_t vacuum_pump_params[] = {
    {
        .gpio_enable = GPIO_PIN(PORT_C, 13),
        .gpio_test = GPIO_PIN(PORT_C, 0),
    },
    {
        .gpio_enable = GPIO_PIN(PORT_C, 14),
        .gpio_test = GPIO_PIN(PORT_C, 1),
    },
    {
        .gpio_enable = GPIO_PIN(PORT_C, 15),
        .gpio_test = GPIO_PIN(PORT_C, 2),
    },
};

#ifdef __cplusplus
}
#endif

/** @} */
