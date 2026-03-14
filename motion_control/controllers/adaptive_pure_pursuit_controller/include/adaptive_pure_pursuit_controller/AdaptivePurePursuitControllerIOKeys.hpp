// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    adaptive_pure_pursuit_controller
/// @{
/// @file
/// @brief      Adaptive Pure Pursuit controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for AdaptivePurePursuitController.
struct AdaptivePurePursuitControllerIOKeys
{
    // Input keys
    etl::string_view current_pose_x; ///< key for current X coordinate
    etl::string_view current_pose_y; ///< key for current Y coordinate
    etl::string_view current_pose_O; ///< key for current orientation
    etl::string_view
        linear_current_speed; ///< key for current linear speed (for adaptive lookahead)
    etl::string_view
        angular_current_speed; ///< key for current angular speed (for acceleration limiting)
    etl::string_view
        motion_direction; ///< key for motion direction (forward_only/backward_only/bidirectional)
    etl::string_view bypass_final_orientation; ///< key for bypassing final orientation (optional)

    // Output keys
    etl::string_view linear_speed_order; ///< key for linear speed order output
    etl::string_view
        angular_speed_order;          ///< key for angular speed order output (FOLLOWING_PATH only)
    etl::string_view pose_reached;    ///< key for pose reached status output
    etl::string_view path_complete;   ///< key for path complete flag output
    etl::string_view is_intermediate; ///< key for intermediate waypoint flag output

    // Output keys for rotation states (pose loop)
    etl::string_view angular_pose_error; ///< key for angular pose error (for ProfileFeedforward)
    etl::string_view recompute_angular_profile; ///< key to trigger angular profile recomputation
    etl::string_view
        rotating_in_place; ///< key for rotation flag (ROTATING_TO_DIRECTION or ROTATING_TO_FINAL)
};

} // namespace motion_control

} // namespace cogip

/// @}
