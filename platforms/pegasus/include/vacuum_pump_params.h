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

#include "vacuum_pump.h"
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Vacuum pump configuration
 */
static const vacuum_pump_params_t vacuum_pump_params[] = {
    {
        .gpio_enable = GPIO_VACUUM1_ENABLE,
        .gpio_test = GPIO_VACUUM1_TEST,
    },
    {
        .gpio_enable = GPIO_VACUUM2_ENABLE,
        .gpio_test = GPIO_VACUUM2_TEST,
    },
    {
        .gpio_enable = GPIO_VACUUM3_ENABLE,
        .gpio_test = GPIO_VACUUM2_TEST,
    },
};

#define VACUUM_PUMP_NUMOF   ARRAY_SIZE(vacuum_pump_params)

#ifdef __cplusplus
}
#endif

/** @} */
