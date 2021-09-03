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

#include "obstacles/obstacles.hpp"

// System includes
#include <list>

// Project includes
#include "cogip_defs/Coords.hpp"
#include "cogip_defs/Polygon.hpp"

namespace cogip {

namespace obstacles {

/// A polygon obstacle defined by the list of points
class Polygon : public Obstacle, public cogip_defs::Polygon {
public:
    /// Constructor
    Polygon(
        const std::list<cogip_defs::Coords> *points = nullptr ///< [in] list of points defining th polygon (optional)
        );

    bool is_point_inside(const cogip_defs::Coords &p) const override;
    bool is_segment_crossing(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const override;
    cogip_defs::Coords nearest_point(const cogip_defs::Coords &p) const override;
    void print_json(cogip::tracefd::File &out) const override;
};

} // namespace obstacles

} // namespace cogip

/// @}
