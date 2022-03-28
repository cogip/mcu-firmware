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

#include "PB_Obstacle.hpp"

#ifndef OBSTACLES_MAX_NUMBER
#  define OBSTACLES_MAX_NUMBER 32  ///< Maximum number of obstacles
#endif

namespace cogip {

namespace obstacles {

/// List of obstacles.
class List: public std::vector<Obstacle *> {
public:
    /// Protobuf message type. Shortcut for original template type.
    using PB_Message = EmbeddedProto::RepeatedFieldFixedSize<Obstacle::PB_Message, OBSTACLES_MAX_NUMBER>;

    /// Constructor.
    List();

    /// Destructor.
    ~List();

    /// Lock the list to avoid using/modifying it from several thread at the same time.
    void lock() { mutex_.lock(); };

    /// Unlock the list.
    void unlock() { mutex_.unlock(); };

    /// Print all obstacles from the list in JSON format.
    void print_json(
        cogip::tracefd::File &out             ///< [out] trace file descriptor
        ) const;

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Message &message                 ///< [out] Protobuf message to fill
        ) const;

    /// Delete all obstacles from the list.
    void clear();

private:
    riot::mutex mutex_;                       ///< mutex protecting list access
};

} // namespace obstacles

} // namespace cogip

/// @}
