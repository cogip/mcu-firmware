// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_path
/// @{
/// @file
/// @brief       Path declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "etl/vector.h"

#include "path/Pose.hpp"

namespace cogip {

namespace path {

#ifndef PATH_MAX_POSES
#define PATH_MAX_POSES 16
#endif

using Poses = etl::vector<Pose, PATH_MAX_POSES>;

/// Path class.
class Path : public Poses {
public:
    /// Constuctor.
    Path(
        const Poses & poses={},  ///< list of positions defining the path
        bool play_in_loop=false             ///< play path in loop
        ) : Poses(poses), play_in_loop_(play_in_loop), current_pose_index_(0) {};

    /// Destructor
    virtual ~Path() {};

    /// Return current position.
    virtual const cogip::path::Pose *current_pose() { return &(data()[current_pose_index_]); };

    /// Reset current position to index 0.
    virtual void reset_current_pose_index() { current_pose_index_ = 0; };

    /// Increment the current position in the robot path (going forward to the next position).
    /// @return new position
    virtual size_t operator++(int);

    /// Decrement the current position in the robot path (going back to the last position).
    /// @return new position
    virtual size_t operator--(int);

    /// Return maximum linear speed to use in order to go to the current position.
    virtual double current_max_speed_linear();

    /// Return maximum angular speed to use in order to go to the current position.
    virtual double current_max_speed_angular();

    /// Mirror all points in path to match the two possible sides of the game.
    virtual void horizontal_mirror_all_poses();

    /// Indicate that the current position is unreachable.
    virtual void unreachable() { next(); };

    /// Go to next position.
    virtual void next() { (*this)++; };

protected:
    bool play_in_loop_;                  ///< play path in loop
    size_t current_pose_index_;          ///< index of the current position in the path
};

} // namespace path

} // namespace cogip

/// @}
