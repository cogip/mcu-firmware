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
#include "utils.h"

/**
 * @brief Compute the distance between two points A and B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  distance between A and B
 */
double collisions_distance_points(const coords_t *a, const coords_t *b);

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
 * @brief Compute polygon center
 *
 * @param[in]   polygon     polygon
 *
 * @return                  polygon center coordinates
 */
coords_t collisions_compute_polygon_center(const polygon_t *polygon);

/**
 * @brief Return the radius of a polygon (the circle radius which contains all
 * polygon points). If center is NULL, the center is computed.
 *
 * @param[in]   polygon     polygon
 * @param[in]   center      center of the polygon
 *
 * @return                  polygon radius
 */
double collisions_compute_polygon_radius(const polygon_t *polygon, const coords_t *center);

/**
 * @brief Check if the given point is inside obstacle, whatever its type.
 *
 * @param[in]   p           point to check
 * @param[in]   obstacle    obstacle
 *
 * @return                  true if point is in polygon, false otherwise
 */
bool collisions_is_point_in_obstacle(const obstacle_t *obstacle, const coords_t *p);

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
bool collisions_is_segment_crossing_line(const coords_t *a, const coords_t *b,
                                         const coords_t *o, const coords_t *p);

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
bool collisions_is_segment_crossing_segment(const coords_t *a, const coords_t *b,
                                            const coords_t *o, const coords_t *p);

/**
 * @brief Check if a line defined by two points A,B is crossing a circle.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   circle      circle
 *
 * @return                  true if (AB) crosses circle, false otherwise
 */
bool collisions_is_line_crossing_circle(const coords_t *a, const coords_t *b,
                                        const circle_t *circle);

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
 * @brief Check if a point C is placed on a segment defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 * @param[in]   c           point C
 *
 * @return                  true if C is on [AB], false otherwise
 */
bool collisions_is_point_on_segment(const coords_t *a, const coords_t *b, const coords_t *c);

/**
 * @brief Compute slope of a line defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  (AB) slope
 */
double collisions_compute_slope(const coords_t *a, const coords_t *b);

/**
 * @brief Compute ordinate of a line defined by two points A,B.
 *
 * @param[in]   a           point A
 * @param[in]   b           point B
 *
 * @return                  (AB) ordinate
 */
double collisions_compute_ordinate(double slope, const coords_t *b);

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
