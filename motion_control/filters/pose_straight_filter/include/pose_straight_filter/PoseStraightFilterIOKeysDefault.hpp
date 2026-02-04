// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup pose_straight_filter Pose straight filter IO keys default values
/// @{
/// @file
/// @brief Default values for Pose straight filter IO keys.
/// @author Generated from PoseStraightFilterIOKeys.hpp

#pragma once

#include "PoseStraightFilterIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Default IO key names for PoseStraightFilter.
/// Each key is set to its corresponding member name as a string literal.
static const PoseStraightFilterIOKeys pose_straight_filter_io_keys_default = {
    // Input keys
    .current_pose_x = "current_pose_x",
    .current_pose_y = "current_pose_y",
    .current_pose_O = "current_pose_O",
    .target_pose_x = "target_pose_x",
    .target_pose_y = "target_pose_y",
    .target_pose_O = "target_pose_O",
    .current_linear_speed = "linear_current_speed",
    .current_angular_speed = "angular_current_speed",
    .target_linear_speed = "linear_target_speed",
    .target_angular_speed = "angular_target_speed",
    .motion_direction = "motion_direction",
    .bypass_final_orientation = "bypass_final_orientation",

    // Output keys
    .linear_pose_error = "linear_pose_error",
    .linear_current_speed = "linear_current_speed",
    .linear_target_speed = "linear_target_speed",
    .linear_speed_filter_flag = "linear_speed_filter_flag",
    .angular_pose_error = "angular_pose_error",
    .angular_current_speed = "angular_current_speed",
    .angular_target_speed = "angular_target_speed",
    .angular_speed_filter_flag = "angular_speed_filter_flag",
    .pose_reached = "pose_reached",
    .current_state = "pose_straight_filter_state",
    .linear_recompute_profile = "linear_recompute_profile",
    .linear_invalidate_profile = "linear_invalidate_profile",
    .angular_recompute_profile = "angular_recompute_profile",
    .angular_invalidate_profile = "angular_invalidate_profile",
    .linear_speed_pid_reset = "linear_speed_pid_reset",
    .angular_speed_pid_reset = "angular_speed_pid_reset",
    .linear_pose_pid_reset = "linear_pose_pid_reset",
    .angular_pose_pid_reset = "angular_pose_pid_reset"};

} // namespace motion_control

} // namespace cogip

/// @}
