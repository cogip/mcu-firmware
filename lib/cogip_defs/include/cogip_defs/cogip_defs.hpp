/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_cogip-defs Module containing common definitions and types used by other modules
 * @ingroup     lib
 * @brief       Common definitions and types
 *
 * @{
 * @file
 * @brief       Main header of the module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */
#pragma once

/* Standard includes */
#include <cstddef>
#include <cstdint>

/**
 * @brief   Polar type
 */
typedef struct {
    double distance;    /**< distance */
    double angle;       /**< angle */
} polar_t;

/** @} */
