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
#include "obstacles.h"

/**
 * @brief Compute the distance between two points A and B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  distance between A and B
 */
double collisions_distance_points(const pose_t *a, const pose_t *b);

/**
 * @brief Check if the given point is inside a circle.
 *
 * @param[in]   p           point to check
 * @param[in]   center      center of circle
 * @param[in]   radius      radius of circle
 *
 * @return                  true if point is in circle, false otherwise
 */
bool collisions_is_point_in_circle(const pose_t *p, const pose_t *center,
                                   const uint32_t radius);

/**
 * @brief Check if the given point is inside a polygon.
 *
 * @param[in]   p           point to check
 * @param[in]   polygon     polygon
 *
 * @return                  true if point is in polygon, false otherwise
 */
bool collisions_is_point_in_polygon(const pose_t *p, const polygon_t *polygon);

/**
 * @brief Check if the given point is inside obstacle, whatever its type.
 *
 * @param[in]   p           point to check
 * @param[in]   obstacle    obstacle
 *
 * @return                  true if point is in polygon, false otherwise
 */
bool collisions_is_point_in_obstacle(const obstacle_t *obstacle, const pose_t *p);

/**
 * @brief Check if a segment defined by two points A,B is crossing line
 * defined by two other points C,D.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 * @param[in]   d           point D
 *
 * @return                  true if [AB] crosses (CD), false otherwise
 */
bool collisions_is_segment_crossing_line(const pose_t *a, const pose_t *b,
                                         const pose_t *o, const pose_t *p);

/**
 * @brief Check if a segment defined by two points A,B is crossing segment
 * defined by two other points C,D.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 * @param[in]   d           point D
 *
 * @return                  true if [AB] crosses [CD], false otherwise
 */
bool collisions_is_segment_crossing_segment(const pose_t *a, const pose_t *b,
                                            const pose_t *o, const pose_t *p);

/**
 * @brief Check if a line defined by two points A,B is crossing a circle.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   center      center of circle
 * @param[in]   radius      radius of circle
 *
 * @return                  true if (AB) crosses circle, false otherwise
 */
bool collisions_is_line_crossing_circle(const pose_t *a, const pose_t *b,
                                        const pose_t *center, const uint32_t radius);

/**
 * @brief Check if a segment defined by two points A,B is crossing a circle.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   center      center of circle
 * @param[in]   radius      radius of circle
 *
 * @return                  true if [AB] crosses circle, false otherwise
 */
bool collisions_is_segment_crossing_circle(const pose_t *a, const pose_t *b,
                                           const pose_t *center, const uint32_t radius);

/**
 * @brief Check if a point C is placed on a segment defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 *
 * @return                  true if C is on [AB], false otherwise
 */
bool collisions_is_point_on_segment(const pose_t *a, const pose_t *b, const pose_t *c);

/**
 * @brief Compute slope of a line defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  (AB) slope
 */
double collisions_compute_slope(const pose_t *a, const pose_t *b);

/**
 * @brief Compute ordinate of a line defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  (AB) ordinate
 */
double collisions_compute_ordinate(double slope, const pose_t *b);

/** @} */
