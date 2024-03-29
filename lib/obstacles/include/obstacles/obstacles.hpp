// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    lib_obstacles Obstacles module
/// @ingroup     lib
/// @brief       Obstacles module
/// @{
/// @file
/// @brief       Public API for obstacles module
/// @author      Eric Courtois <eric.courtois@gmail.com>
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// System includes
#include "etl/set.h"

// Project includes
#include "cogip_defs/Coords.hpp"
#include "obstacles/Obstacle.hpp"
#include "obstacles/List.hpp"

#ifndef OBSTACLE_MAX_LISTS
#  define OBSTACLE_MAX_LISTS 6  /**< maximun numver of obstacle lists */
#endif

namespace cogip {

namespace obstacles {

extern etl::set<List const *, OBSTACLE_MAX_LISTS> all_obstacles;  ///< List of all obstacle lists

/// Check if the given point is inside an obstacle.
/// @return                  true if point is inside, false otherwise
bool is_point_in_obstacles(
    const cogip_defs::Coords &p, ///< [in] point to check
    const Obstacle *filter       ///< [in] obstacle to filter
    );

} // namespace obstacles

} // namespace cogip

/// @}
