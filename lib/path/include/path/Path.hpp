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

#include <vector>

#include "path/Pose.hpp"

namespace cogip {

namespace path {

/// Path class.
class Path : public std::vector<Pose> {
public:
    /// Constuctor.
    Path(
        const std::vector<Pose> &poses,  ///< list of positions defining the path
        bool play_in_loop=false          ///< play path in loop
        ) : std::vector<Pose>(poses), play_in_loop_(play_in_loop), current_pose_index_(0) {};

    /// Return current position.
    const cogip::path::Pose &current_pose() const { return at(current_pose_index_); };

    /// Reset current position to index 0.
    void reset_current_pose_index() { current_pose_index_ = 0; };

    /// Increment the current position in the robot path (going forward to the next position).
    /// @return new position
    size_t operator++(int);

    /// Decrement the current position in the robot path (going back to the last position).
    /// @return new position
    size_t operator--(int);

    /// Return maximum linear speed to use in order to go to the current position.
    double current_max_speed_linear();

    /// Return maximum angular speed to use in order to go to the current position.
    double current_max_speed_angular();

    /// Mirror all points in path to match the two possible sides of the game.
    void horizontal_mirror_all_poses();

private:
    bool play_in_loop_;                  ///< play path in loop
    size_t current_pose_index_;          ///< index of the current position in the path
};

} // namespace path

} // namespace cogip

/// @}
