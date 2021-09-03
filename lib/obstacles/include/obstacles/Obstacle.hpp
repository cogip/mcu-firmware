// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_obstacles
/// @{
/// @file
/// @brief       Polygon obstacle class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// System includes
#include <cstdint>

// Project includes
#include "cogip_defs/Coords.hpp"
#include "cogip_defs/Polygon.hpp"
#include "tracefd/File.hpp"

namespace cogip {

namespace obstacles {

/// An obstacle used to detect and avoid collisions.
class Obstacle {
public:
    /// Constructor
    Obstacle(
        const cogip_defs::Coords &center, ///< [in] obstacle center
        double radius,                    ///< [in] obstacle circumscribed circle radius
        double angle                      ///< [in] absolute angle
        );

    /// Destructor
    virtual ~Obstacle() {};

    /// Return bounding box of an obstacle. This bounding box has nb_vertices
    /// and has a radius of ((1 + radius_margin) * radius).
    /// @return bounding box polygon
    cogip_defs::Polygon bounding_box(
        const uint8_t nb_vertices,        ///< [in] number of bounding box vertices
        double radius_margin              ///< [in] radius margin
        ) const;

    /// Check if the given point is inside the obstacle.
    /// @return true if point is inside, false otherwise
    virtual bool is_point_inside(
        const cogip_defs::Coords &p       ///< [in] point to check
        ) const = 0;

    /// Check if a segment defined by two points A,B is crossing an obstacle.
    /// @return true if [AB] crosses obstacle, false otherwise
    virtual bool is_segment_crossing(
        const cogip_defs::Coords &a,      ///< [in] point A
        const cogip_defs::Coords &b       ///< [in] point B
        ) const = 0;

    /// Find the nearest point of obstacle perimeter from given point.
    /// @return position of nearest point
    virtual cogip_defs::Coords nearest_point(
        const cogip_defs::Coords &p       ///< [in] point to check
        ) const = 0;

    /// Print obstacles in JSON format.
    virtual void print_json(
        cogip::tracefd::File &out         ///< [out] trace file descriptor
        ) const = 0;

    /// Return obstacle center.
    const cogip_defs::Coords &center() const { return center_; };

    /// Return obstacle circumscribed circle radius.
    double radius() const { return radius_; };

    /// Return absolute angle.
    double angle() const { return angle_; };

protected:
    cogip_defs::Coords center_;           ///< obstacle center
    double radius_;                       ///< obstacle circumscribed circle radius
    double angle_;                        ///< absolute angle
};

} // namespace obstacles

} // namespace cogip

/// @}
