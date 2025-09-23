// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup pose_pid_controller Passthrough Pose PID controller IO keys default values
/// @{
/// @file
/// @brief Default values for Passthrough Pose PID controller IO keys.
/// @author Generated from PassthroughPosePIDControllerIOKeys.hpp

#pragma once

#include "PassthroughPosePIDControllerIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Default IO key names for linear PassthroughPosePIDController.
/// Each key is prefixed with "linear_" and set to its corresponding member name.
static const PassthroughPosePIDControllerIOKeys linear_passthrough_pose_pid_controller_io_keys_default = {
    .position_error = "linear_position_error",
    .speed_order = "linear_speed_order",
    .target_speed = "linear_target_speed"
};

/// @brief Default IO key names for angular PassthroughPosePIDController.
/// Each key is prefixed with "angular_" and set to its corresponding member name.
static const PassthroughPosePIDControllerIOKeys angular_passthrough_pose_pid_controller_io_keys_default = {
    .position_error = "angular_position_error",
    .speed_order = "angular_speed_order",
    .target_speed = "angular_target_speed"
};

} // namespace motion_control

} // namespace cogip

/// @}
