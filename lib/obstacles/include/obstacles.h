/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_obstacles Obstacles module
 * @ingroup     lib
 * @brief       Obstacles module
 *
 *
 * @{
 * @file
 * @brief       Public API for obstacles module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* RIOT includes */
#include "mutex.h"

/* Project includes */
#include "cogip_defs.h"
#include "tracefd.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OBSTACLES_NUMOF
#define OBSTACLES_NUMOF 1   /**< number of used obstacles contexts */
#endif

#ifndef OBSTACLES_MAX_NUMBER
#define OBSTACLES_MAX_NUMBER    360
#endif

/**
 * @brief   Obstacles parameters
 */
typedef struct {
    uint32_t default_circle_radius;     /**< obstacle default radius */
    uint32_t default_rectangle_width;   /**< obstacle of rectangle type default width */
    uint32_t min_distance;              /**< minimun distance from origin to create an obstacle */
    uint32_t max_distance;              /**< maximum distance from origin to create an obstacle */
} obstacles_params_t;

/**
 * @brief   Obstacle types
 */
typedef enum {
    OBSTACLE_CIRCLE,            /**< circle */
    OBSTACLE_POLYGON,           /**< polygon */
    OBSTACLE_RECTANGLE,         /**< rectangle */
} obstacle_type_t;

/**
 * Obstacle default definition
 */
typedef struct obstacle_t obstacle_t;

/**
 * @brief    Generic obstacle definition
 */
struct obstacle_t {
    obstacle_type_t type;           /**< obstacle type */
    coords_t center;                /**< obstacle center */
    double radius;                  /**< obstacle circumscribed circle radius */
    double angle;                   /**< absolute angle */
    union {
        circle_t circle;
        polygon_t polygon;
        rectangle_t rectangle;
    } form;                         /**< geometric form */
};

/**
 * @brief   Default obstacles identifier definition
 */
typedef unsigned int obstacles_t;

/**
 * @brief   Default AABB obstacles table identifier definition
 */
typedef unsigned int obstacles_aabb_id_t;

/**
 * @brief   Default dynamic obstacles table identifier definition
 */
typedef unsigned int obstacles_dyn_id_t;

/**
 * @brief Initialize obstacles context
 *
 * @param[in]   obstacles_params  obstacles parameters
 *
 * @return                        id of the initialized obstacles
 */
obstacles_t obstacles_init(const obstacles_params_t *obstacles_params);

/**
 * @brief Initialize a rectangle obstacle
 *
 * @param[in]   center      obstacle center
 * @param[in]   length_x    obstacle length on x axis
 * @param[in]   length_y    obstacle length on y axis
 * @param[in]   angle       obstacle rotation angle
 *
 * @return                  created obstacle
 */
obstacle_t obstacles_rectangle_init(coords_t center, double length_x,
                                    double length_y, double angle);

/**
 * @brief Return bounding box of an obstacle. This bounding box has nb_vertices
 * and has a radius of ((1 + radius_margin) * radius).
 *
 * @param[in]   obstacle        obstacle
 * @param[in]   nb_vertices     number of bounding box vertices
 * @param[in]   radius_margin   radius margin
 *
 * @return                      bounding box polygon
 */
polygon_t obstacles_compute_obstacle_bounding_box(const obstacle_t *obstacle,
                                                  const uint8_t nb_points, double radius_margin);

/**
 * @brief Return the obstacle circumscribed circle radius
 * @param[in]   obstacle      obstacle
 * @return                    radius
 */
double obstacles_compute_radius(const obstacle_t *obstacle);

/**
 * @brief Return the default circle obstacle radius
 * @param[in]   obstacles_id  obstacles id
 * @return                    radius
 */
double obstacles_get_default_circle_radius(const obstacles_t obstacles_id);

/**
 * @brief Return the minimal distance of obstacle detection
 * @param[in]   obstacles_id  obstacles id
 * @return                    minimal distance
 */
uint32_t obstacles_get_min_distance(const obstacles_t obstacles_id);

/**
 * @brief Return the maximal distance of obstacle detection
 * @param[in]   obstacles_id  obstacles id
 * @return                    maximal distance
 */
uint32_t obstacles_get_max_distance(const obstacles_t obstacles_id);

/**
 * @brief Return the number of obstacles
 * @param[in]   obstacles_id  obstacles id
 * @return                    number of obstacles
 */
size_t obstacles_get_nb_obstacles(const obstacles_t obstacles_id);

/**
 * @brief Reset the list of obstacles
 * @param[in]   obstacles_id  obstacles id
 * @return
 */
void obstacles_reset(const obstacles_t obstacles_id);

/**
 * @brief Get an obstacle
 * @param[in]   obstacles_id  obstacles id
 * @param[in]   n             index of the obstacle to get
 * @return                    pointer to the nth obstacle
 */
const obstacle_t *obstacles_get(const obstacles_t obstacles_id, size_t n);

/**
 * @brief Get all obstacles
 * @param[in]   obstacles_id  obstacles id
 * @return                    pointer the list of obstacles
 */
const obstacle_t *obstacles_get_all(const obstacles_t obstacles_id);

/**
 * @brief Add an obstacle
 * @param[in]   obstacles_id  obstacles id
 * @return                    true if the obstacle was successfully added, false otherwise
 */
bool obstacles_add(const obstacles_t obstacles_id, const obstacle_t obstacle);

/**
 * @brief Lock obstacles data
 * @param[in]   obstacles_id  obstacles id to initialize
 */
void obstacles_lock(const obstacles_t obstacles_id);

/**
 * @brief Unlock obstacles data
 * @param[in]   obstacles_id  obstacles id
 */
void obstacles_unlock(const obstacles_t obstacles_id);

/**
 * @brief Check if the given point is inside the obstacle
 *
 * @param[in]   obstacle    obstacle
 * @param[in]   p           point to check
 *
 * @return                  true if point is inside, false otherwise
 */
bool obstacles_is_point_in_obstacle(const obstacle_t *obstacle, const coords_t *p);

/**
 * @brief Check if the given point is inside an obstacle
 *
 * @param[in]   p           point to check
 * @param[in]   filter      obstacle to filter
 *
 * @return                  true if point is inside, false otherwise
 */
bool obstacles_is_point_in_obstacles(const coords_t *p, const obstacle_t *filter);

/**
 * @brief Check if a segment defined by two points A,B is crossing an obstacle.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   obstacle    obstacle
 *
 * @return                  true if [AB] crosses obstacle, false otherwise
 */
bool obstacles_is_segment_crossing_obstacle(const coords_t *a, const coords_t *b, const obstacle_t *obstacle);

/**
 * @brief Find the nearest point of obstacle perimeter from given point.
 *
 * @param[in]   obstacle    obstacle
 * @param[in]   p           point to check
 *
 * @return                  position of nearest point
 */
coords_t obstacles_find_nearest_point_in_obstacle(const obstacle_t *obstacle,
                                                  const coords_t *p);

/**
 * @brief Print specified obstacles in JSON format
 * @param[in]   obstacles_id  obstacles id
 * @param[in]   out           tracefd descriptor used to print obstacles
 */
void obstacles_print_json(const obstacles_t obstacles_id, tracefd_t out);

/**
 * @brief Print all obstacles in JSON format
 * @param[in]   out           tracefd descriptor used to print obstacles
 */
void obstacles_print_all_json(tracefd_t out);

#ifdef __cplusplus
}
#endif

/** @} */
