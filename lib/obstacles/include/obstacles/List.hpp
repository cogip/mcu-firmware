// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_obstacles
/// @{
/// @file
/// @brief       List obstacles class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Standard includes
#include <cstdint>
#include <vector>

// RIOT includes
#include "native_sched.h"
#include "riot/mutex.hpp"

// Project includes
#include "obstacles/Obstacle.hpp"
#include "tracefd/File.hpp"

namespace cogip {

namespace obstacles {

/// List of obstacles.
class List: public std::vector<Obstacle *> {
public:
    /// Constructor.
    List(
        uint32_t default_circle_radius = 0,   ///< [in] default radius for circle obstacles
        uint32_t default_rectangle_width = 0, ///< [in] default width for rectangle obstacles
        uint32_t min_distance = 0,            ///< [in] minium distance to a valid obstacle
        uint32_t max_distance = 0             ///< [in] maximum distance to a valid obstacle
        );

    /// Destructor.
    ~List();

    /// Return the default radius for circle obstacles.
    uint32_t default_circle_radius() { return default_circle_radius_; };

    /// Return the default width for rectangle obstacles.
    uint32_t default_rectangle_width() { return default_rectangle_width_; };

    /// Return the minimum distance to a valid obstacle.
    uint32_t min_distance() { return min_distance_; };

    /// Return the maximum distance to a valid obstacle.
    uint32_t max_distance() { return max_distance_; };

    /// Lock the list to avoid using/modifying it from several thread at the same time.
    void lock() { mutex_.lock(); };

    /// Unlock the list.
    void unlock() { mutex_.unlock(); };

    /// Print all obstacles from the list in JSON format.
    void print_json(
        cogip::tracefd::File &out             ///< [out] trace file descriptor
        ) const;

    /// Delete all obstacles from the list.
    void clear();

private:
    uint32_t default_circle_radius_;          ///< obstacle default radius
    uint32_t default_rectangle_width_;        ///< obstacle of rectangle type default width
    uint32_t min_distance_;                   ///< minimun distance from origin to create an obstacle
    uint32_t max_distance_;                   ///< maximum distance from origin to create an obstacle
    riot::mutex mutex_;                       ///< mutex protecting list access
};

} // namespace obstacles

} // namespace cogip

/// @}
