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
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POLY_MAX                16  /**< Maximum number of polygons */
#define POLY_MAX_POINTS         6   /**< Maximum number of vertices composing a polygon */


/**
 * @brief   Position type
 */
typedef struct {
    double x;   /**< x-position */
    double y;   /**< y-position */
    double O;   /**< 0-orientation */
} pose_t;


/**
 * @brief   Polar type
 */
typedef struct {
    double distance;    /**< distance */
    double angle;       /**< angle */
} polar_t;

/**
 * @brief   Vector type
 */
typedef struct {
    double x;   /**< x-position */
    double y;   /**< y-position */
} vector_t;

/**
 * @brief   Polygon type
 */
typedef struct {
    uint8_t count;                  /**< number of vertices in the polygon */
    pose_t points[POLY_MAX_POINTS]; /**< vertices defining the polygon */
} polygon_t;

/**
 * @brief   Circle type
 */
typedef struct {
    pose_t center;                  /**< circle center */
    double radius;                  /**< circle radius */
} circle_t;

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
               || (p1->x == p2->x && p1->y == p2->y && p1->O == p2->O);
    }
    else {
        return 0;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

/** @} */
