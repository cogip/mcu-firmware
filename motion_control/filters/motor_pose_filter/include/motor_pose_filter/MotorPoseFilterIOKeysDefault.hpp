// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup motor_pose_filter Motor pose filter IO keys default values
/// @{
/// @file
/// @brief Default values for Motor pose filter IO keys.
/// @author Generated from MotorPoseFilterIOKeys.hpp

#pragma once

#include "MotorPoseFilterIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Default IO key names for MotorPoseFilter.
/// Each key is set to its corresponding member name as a string literal.
static const MotorPoseFilterIOKeys motor_pose_filter_io_keys_default = {
    .current_pose = "current_pose",
    .target_pose = "target_pose",
    .current_speed = "current_speed",
    .target_speed = "target_speed",
    .pose_reached = "pose_reached",

    .position_error = "position_error",
    .filtered_speed = "filtered_speed",
    .speed_filter_flag = "speed_filter_flag",
    .pose_reached_out = "pose_reached_out"
};

} // namespace motion_control

} // namespace cogip

/// @}
