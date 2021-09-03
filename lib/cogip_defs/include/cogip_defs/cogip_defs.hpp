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

#include "cogip_defs/Coords.hpp"

/**
 * @brief   Polar type
 */
typedef struct {
    double distance;    /**< distance */
    double angle;       /**< angle */
} polar_t;

/**
 * @brief   Position type
 */
typedef struct {
    cogip::cogip_defs::Coords coords;    /**< coordinates */
    double O;                            /**< 0-orientation */
} pose_t;

/**
 * @brief Check equality of the pose
 *
 * @param[in]   p1   first pose to compare
 * @param[in]   p2   second pose to compare
 *
 * @return              1 if poses are equal, not 0 otherwise
 */
static inline int pose_equal(const pose_t *p1, const pose_t *p2)
{
    if ((p1 != NULL) && (p2 != NULL)) {
        return p1 == p2
               || (p1->coords.x() == p2->coords.x() && p1->coords.y() == p2->coords.y() && p1->O == p2->O);
    }
    else {
        return 0;
    }
}

/** @} */
