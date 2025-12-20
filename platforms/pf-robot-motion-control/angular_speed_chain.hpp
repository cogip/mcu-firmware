// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Angular speed chain controller instances for speed PID tuning
/// @details Simplified chain that generates a trapezoidal velocity profile
///          and feeds it to the speed PID controller.
///          Chain: PoseErrorFilter -> ProfileFeedforwardController -> SpeedPIDController

#pragma once

#include "app_conf.hpp"
#include "pose_error_filter/PoseErrorFilter.hpp"
#include "pose_error_filter/PoseErrorFilterIOKeys.hpp"
#include "pose_error_filter/PoseErrorFilterParameters.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "quadpid_chain.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace angular_speed_chain {

// ============================================================================
// PoseErrorFilter for computing angle difference to target
// ============================================================================

inline cogip::motion_control::PoseErrorFilterIOKeys pose_error_filter_io_keys = {
    .target_x = "",  // Not used in ANGULAR mode
    .target_y = "",  // Not used in ANGULAR mode
    .target_O = "target_pose_O",
    .current_x = "", // Not used in ANGULAR mode
    .current_y = "", // Not used in ANGULAR mode
    .current_O = "current_pose_O",
    .pose_error = "angular_pose_error",
    .recompute = "angular_speed_recompute_profile",
    .pose_reached = "pose_reached"};

inline cogip::motion_control::PoseErrorFilterParameters
    pose_error_filter_parameters(cogip::motion_control::PoseErrorFilterMode::ANGULAR,
                                 angular_threshold  // pose_reached_threshold
    );

inline cogip::motion_control::PoseErrorFilter
    pose_error_filter(pose_error_filter_io_keys, pose_error_filter_parameters);

// ============================================================================
// ProfileFeedforwardController for velocity profile generation
// ============================================================================

/// IO keys: pose_error triggers the profile, feedforward_velocity is the output
inline cogip::motion_control::ProfileFeedforwardControllerIOKeys profile_feedforward_io_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .recompute_profile = "angular_speed_recompute_profile",
    .invalidate_profile = "",
    .feedforward_velocity = "angular_speed_order", // Direct to speed PID
    .tracking_error = "angular_speed_tracking_error"};

inline cogip::motion_control::ProfileFeedforwardControllerParameters
    profile_feedforward_parameters(platform_max_speed_angular_deg_per_period, // max_speed
                                   platform_max_acc_angular_deg_per_period2,  // acceleration
                                   platform_max_dec_angular_deg_per_period2,  // deceleration
                                   true                                       // must_stop_at_end
    );

inline cogip::motion_control::ProfileFeedforwardController
    profile_feedforward_controller(profile_feedforward_io_keys, profile_feedforward_parameters);

// ============================================================================
// SpeedPIDController
// ============================================================================

inline cogip::motion_control::SpeedPIDController
    speed_controller(cogip::motion_control::angular_speed_pid_controller_io_keys_default,
                     quadpid_chain::angular_speed_controller_parameters);

} // namespace angular_speed_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
