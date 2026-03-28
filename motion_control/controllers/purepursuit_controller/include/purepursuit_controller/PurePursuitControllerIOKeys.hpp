// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup
/// @{
/// @file
/// @brief    PurePursuit controller IO keys
/// @author

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for PurePursuitControllerIOKeys.
struct PurePursuitControllerIOKeys
{
    // Input keys
    etl::string_view current_pose_x; ///< key for current X coordinate
    etl::string_view current_pose_y; ///< key for current Y coordinate
    etl::string_view current_pose_O; ///< key for current orientation

    // Output keys
    etl::string_view linear_speed_order;  ///< key for linear speed order output
    etl::string_view angular_speed_order; ///< key for angular speed order output
    etl::string_view linear_pose_error;   ///< key for linear pose error output (distance to goal)
    etl::string_view angular_pose_error;  ///< key for angular pose error output (angle to carrot)
    etl::string_view pose_reached;             ///< key for pose reached status output
    etl::string_view linear_pid_reset;         ///< key for linear PID reset signal
    etl::string_view angular_pid_reset;        ///< key for angular PID reset signal
    etl::string_view linear_ramp_reset;        ///< key for linear ramp filter reset signal
    etl::string_view angular_ramp_reset;       ///< key for angular ramp filter reset signal
};

} // namespace motion_control

} // namespace cogip

/// @}
