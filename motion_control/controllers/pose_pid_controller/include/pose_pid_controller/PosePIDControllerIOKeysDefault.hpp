// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup pose_pid_controller Pose PID controller IO keys default values
/// @{
/// @file
/// @brief Default values for Pose PID controller IO keys.
/// @author Generated from PosePIDControllerIOKeys.hpp

#pragma once

#include "PosePIDControllerIOKeys.hpp"

namespace cogip {
namespace motion_control {

/// @brief Default IO key names for linear PosePIDController.
/// Each key is prefixed with "linear_" and set to its corresponding member
/// name.
static const PosePIDControllerIOKeys linear_pose_pid_controller_io_keys_default = {
    .position_error = "linear_pose_error",
    .current_speed = "linear_current_speed",
    .target_speed = "linear_target_speed",
    .disable_filter = "linear_disable_filter",
    .pose_reached = "linear_pose_reached",
    .speed_order = "linear_speed_order"};

/// @brief Default IO key names for angular PosePIDController.
/// Each key is prefixed with "angular_" and set to its corresponding member
/// name.
static const PosePIDControllerIOKeys angular_pose_pid_controller_io_keys_default = {
    .position_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .target_speed = "angular_target_speed",
    .disable_filter = "angular_disable_filter",
    .pose_reached = "angular_pose_reached",
    .speed_order = "angular_speed_order"};

} // namespace motion_control
} // namespace cogip

/// @}
