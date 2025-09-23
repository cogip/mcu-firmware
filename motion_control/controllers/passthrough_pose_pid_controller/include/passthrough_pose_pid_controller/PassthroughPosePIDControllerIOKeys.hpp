// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_pid_controller Pose PID controller IO keys
/// @{
/// @file
/// @brief      Passthrough pose PID controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// System includes
#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a PassthroughPosePIDController.
struct PassthroughPosePIDControllerIOKeys {
    etl::string_view position_error;  ///< e.g. "linear_position_error"  or "angular_position_error"
    etl::string_view speed_order;     ///< e.g. "linear_speed_order"     or "angular_speed_order"
    etl::string_view target_speed;    ///< e.g. "linear_target_speed"    or "angular_target_speed"
};

}  // namespace motion_control

}  // namespace cogip

/// @}
