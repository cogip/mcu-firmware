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

// RIOT includes
#include <mutex.h>
#include "native_sched.h"

// Project includes
#include "obstacles/Obstacle.hpp"

#ifndef OBSTACLES_MAX_NUMBER
#  define OBSTACLES_MAX_NUMBER 32  ///< Maximum number of obstacles
#endif

namespace cogip {

namespace obstacles {

/// List of obstacles.
class List: public etl::vector<Obstacle *, OBSTACLES_MAX_NUMBER> {
public:
    /// Constructor.
    List();

    /// Destructor.
    ~List();

    /// Lock the list to avoid using/modifying it from several thread at the same time.
    void lock() { mutex_lock(&mutex_); };

    /// Unlock the list.
    void unlock() { mutex_unlock(&mutex_); };

    /// Return number of enabled obstacles.
    size_t enabled_obstacles() const;

private:
    mutex_t mutex_ = MUTEX_INIT;            ///< mutex protecting list access
};

} // namespace obstacles

} // namespace cogip

/// @}
