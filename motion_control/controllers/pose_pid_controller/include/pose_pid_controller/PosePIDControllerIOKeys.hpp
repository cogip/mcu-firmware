// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_pid_controller Pose PID controller IO keys
/// @{
/// @file
/// @brief      Pose PID controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a PosePIDController.
struct PosePIDControllerIOKeys {
    etl::string_view position_error;   ///< e.g. "pose_error"
    etl::string_view current_speed;    ///< e.g. "current_speed"
    etl::string_view target_speed;     ///< e.g. "target_speed"
    etl::string_view disable_filter;   ///< e.g. "disable_speed_filter"
    etl::string_view pose_reached;     ///< e.g. "pose_reached"
    etl::string_view speed_order;      ///< e.g. "speed_order"
};

}  // namespace motion_control

}  // namespace cogip

/// @}
