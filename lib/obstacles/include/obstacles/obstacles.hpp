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
 * @{
 * @file
 * @brief       Public API for obstacles module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

// Standard includes
#include <cstdint>

// Project includes
#include "cogip_defs/Polygon.hpp"
#include "tracefd/File.hpp"

#if 0 // Unused?
/**
 * @brief   Default AABB obstacles table identifier definition
 */
typedef unsigned int obstacles_aabb_id_t;

/**
 * @brief Return the obstacle circumscribed circle radius
 * @param[in]   obstacle      obstacle
 * @return                    radius
 */
double obstacles_compute_radius(const obstacle_t *obstacle);
#endif

namespace cogip {

namespace obstacles {

class Obstacle {
public:
    /// @brief Constructor
    /// @param[in]   center     obstacle center
    /// @param[in]   radius     obstacle circumscribed circle radius
    /// @param[in]   angle      absolute angle
    Obstacle(const cogip_defs::Coords &center, double radius, double angle);

    /// @brief Destructor
    virtual ~Obstacle() {};

    /// @brief Return bounding box of an obstacle. This bounding box has nb_vertices
    ///        and has a radius of ((1 + radius_margin) * radius).
    /// @param[in]   nb_vertices     number of bounding box vertices
    /// @param[in]   radius_margin   radius margin
    /// @return                      bounding box polygon
    cogip_defs::Polygon bounding_box(const uint8_t nb_vertices, double radius_margin) const;

    /// @brief Check if the given point is inside the obstacle
    /// @param[in]   p               point to check
    /// @return                      true if point is inside, false otherwise
    virtual bool is_point_inside(const cogip_defs::Coords &p) const = 0;

    /// @brief Check if a segment defined by two points A,B is crossing an obstacle.
    /// @param[in]   a               point A
    /// @param[in]   b               point B
    /// @return                      true if [AB] crosses obstacle, false otherwise
    virtual bool is_segment_crossing(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const = 0;

    /// @brief Find the nearest point of obstacle perimeter from given point.
    /// @param[in]   p               point to check
    /// @return                      position of nearest point
    virtual cogip_defs::Coords nearest_point(const cogip_defs::Coords &p) const = 0;

    /// @brief Print obstacles in JSON format
    /// @param[out]   out            Trace file descriptor
    virtual void print_json(cogip::tracefd::File &out) const = 0;

    /// @brief Return obstacle center
    const cogip_defs::Coords &center() const { return center_; };

    /// @brief Return obstacle circumscribed circle radius
    double radius() const { return radius_; };

    /// @brief Return absolute angle
    double angle() const { return angle_; };

protected:
    cogip_defs::Coords center_;      //// obstacle center
    double radius_;                  //// obstacle circumscribed circle radius
    double angle_;                   //// absolute angle
};

/// @brief Check if the given point is inside an obstacle
/// @param[in]   p           point to check
/// @param[in]   filter      obstacle to filter
/// @return                  true if point is inside, false otherwise
bool is_point_in_obstacles(const cogip_defs::Coords &p, const Obstacle *filter);

/// @brief Print all obstacles from all lists
/// @param[in]   out         trace file descriptor
void print_all_json(cogip::tracefd::File &out);

} // namespace obstacles

} // namespace cogip

/** @} */
