// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    profile_feedforward_controller Profile Feedforward controller IO keys
/// @{
/// @file
/// @brief      Profile Feedforward controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for ProfileFeedforwardController.
///
/// This controller generates a trapezoidal velocity profile and computes:
/// - feedforward velocity (from the profile)
/// - position tracking error (actual remaining distance - theoretical remaining distance)
struct ProfileFeedforwardControllerIOKeys
{
    etl::string_view pose_error;           ///< e.g. "linear_pose_error" (distance remaining)
    etl::string_view current_speed;        ///< e.g. "current_linear_speed"
    etl::string_view new_target;           ///< e.g. "new_target" (flag to regenerate profile)
    etl::string_view feedforward_velocity; ///< e.g. "linear_feedforward_velocity" (output)
    etl::string_view tracking_error;       ///< e.g. "linear_tracking_error" (output)
};

} // namespace motion_control

} // namespace cogip

/// @}
