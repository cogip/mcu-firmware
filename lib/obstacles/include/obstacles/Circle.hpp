// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_obstacles
/// @{
/// @file
/// @brief       Circle obstacle class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "obstacles/Obstacle.hpp"

namespace cogip {

namespace obstacles {

/// Circle obstacle defined by its center and radius
class Circle : public Obstacle {
public:
    /// Constructor
    Circle(
        const cogip::cogip_defs::Coords &center, ///< [in] center of the circle
        double radius                            ///< [in] radius of the circle
        );

    bool is_point_inside(const cogip_defs::Coords &p) const override;
    bool is_segment_crossing(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const override;
    cogip_defs::Coords nearest_point(const cogip_defs::Coords &p) const override;

private:
    /// Check if a line defined by two points A,B is crossing a circle.
    /// @return true if (AB) crosses circle, false otherwise
    bool is_line_crossing_circle(
        const cogip_defs::Coords &a, ///< point A
        const cogip_defs::Coords &b  ///< point B
        ) const;
};

} // namespace obstacles

} // namespace cogip

/// @}
