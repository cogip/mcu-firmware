/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lib_collisions
 * @ingroup     lib
 * @brief       2D collisions management
 *
 * @{
 *
 * @file
 * @brief       2D collisions management
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once


/* Project includes */
#include "obstacles.hpp"
#include "utils.h"

/**
 * @brief Check if the given point is inside a circle.
 *
 * @param[in]   circle      circle
 * @param[in]   p           point to check
 *
 * @return                  true if point is in circle, false otherwise
 */
bool collisions_is_point_in_circle(const circle_t *circle, const coords_t *p);

/**
 * @brief Check if the given point is inside a polygon.
 *
 * @param[in]   polygon     polygon
 * @param[in]   p           point to check
 *
 * @return                  true if point is in polygon, false otherwise
 */
bool collisions_is_point_in_polygon(const polygon_t *polygon, const coords_t *p);

/**
 * @brief Check if a segment defined by two points A,B is crossing a circle.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   circle      circle
 *
 * @return                  true if [AB] crosses circle, false otherwise
 */
bool collisions_is_segment_crossing_circle(const coords_t *a, const coords_t *b,
                                           const circle_t *circle);

/**
 * @brief Check if a segment defined by two points A,B is crossing a polygon.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   polygon     polygon
 *
 * @return                  true if (AB) crosses circle, false otherwise
 */
bool collisions_is_segment_crossing_polygon(const coords_t *a, const coords_t *b,
                                            const polygon_t *polygon);

/**
 * @brief Find the nearest point of polygon perimeter from given point.
 *
 * @param[in]   polygon     polygon
 * @param[in]   p           point to check
 *
 * @return                  position of nearest point
 */
coords_t collisions_find_nearest_point_in_polygon(const polygon_t *polygon,
                                                  const coords_t *p);

/**
 * @brief Find the nearest point of circle perimeter from given point.
 *
 * @param[in]   circle      circle
 * @param[in]   p           point to check
 *
 * @return                  position of nearest point
 */
coords_t collisions_find_nearest_point_in_circle(const circle_t *circle,
                                                 const coords_t *p);

/** @} */
