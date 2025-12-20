// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Linear speed chain controller instances for speed PID tuning
/// @details Simplified chain that generates a trapezoidal velocity profile
///          and feeds it to the speed PID controller.
///          Chain: PoseErrorFilter -> ProfileFeedforwardController -> SpeedPIDController ->
///          TuningPoseReachedFilter

#pragma once

#include "app_conf.hpp"
#include "pose_error_filter/PoseErrorFilter.hpp"
#include "pose_error_filter/PoseErrorFilterIOKeys.hpp"
#include "pose_error_filter/PoseErrorFilterParameters.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "quadpid_chain.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilter.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilterIOKeys.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace linear_speed_chain {

// ============================================================================
// PoseErrorFilter for computing distance to target
// ============================================================================

inline cogip::motion_control::PoseErrorFilterIOKeys pose_error_filter_io_keys = {
    .target_x = "target_pose_x",
    .target_y = "target_pose_y",
    .target_O = "", // Not used for error in LINEAR mode
    .current_x = "current_pose_x",
    .current_y = "current_pose_y",
    .current_O = "current_pose_O", // Used for direction (bidirectional)
    .pose_error = "linear_pose_error",
    .recompute = "linear_speed_recompute_profile"};

inline cogip::motion_control::PoseErrorFilterParameters pose_error_filter_parameters(
    cogip::motion_control::PoseErrorFilterMode::LINEAR,
    0.0f // pose_reached_threshold (not used, TuningPoseReachedFilter handles this)
);

inline cogip::motion_control::PoseErrorFilter pose_error_filter(pose_error_filter_io_keys,
                                                                pose_error_filter_parameters);

// ============================================================================
// ProfileFeedforwardController for velocity profile generation
// ============================================================================

/// IO keys: pose_error triggers the profile, feedforward_velocity is the output
inline cogip::motion_control::ProfileFeedforwardControllerIOKeys profile_feedforward_io_keys = {
    .pose_error = "linear_pose_error",
    .current_speed = "linear_current_speed",
    .recompute_profile = "linear_speed_recompute_profile",
    .invalidate_profile = "",
    .feedforward_velocity = "linear_speed_order", // Direct to speed PID
    .tracking_error = "linear_speed_tracking_error",
    .profile_complete = "linear_speed_profile_complete"};

inline cogip::motion_control::ProfileFeedforwardControllerParameters
    profile_feedforward_parameters(platform_max_speed_linear_mm_per_period, // max_speed
                                   platform_max_acc_linear_mm_per_period2,  // acceleration
                                   platform_max_dec_linear_mm_per_period2,  // deceleration
                                   true                                     // must_stop_at_end
    );

inline cogip::motion_control::ProfileFeedforwardController
    profile_feedforward_controller(profile_feedforward_io_keys, profile_feedforward_parameters);

// ============================================================================
// SpeedPIDController
// ============================================================================

inline cogip::motion_control::SpeedPIDController
    speed_controller(cogip::motion_control::linear_speed_pid_controller_io_keys_default,
                     quadpid_chain::linear_speed_controller_parameters);

// ============================================================================
// TuningPoseReachedFilter for signaling pose_reached when profile completes
// ============================================================================

inline cogip::motion_control::TuningPoseReachedFilterIOKeys tuning_pose_reached_filter_io_keys = {
    .profile_complete = "linear_speed_profile_complete", .pose_reached = "pose_reached"};

inline cogip::motion_control::TuningPoseReachedFilter
    tuning_pose_reached_filter(tuning_pose_reached_filter_io_keys);

} // namespace linear_speed_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
