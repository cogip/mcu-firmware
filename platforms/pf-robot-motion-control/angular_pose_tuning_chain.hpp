// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Angular pose chain controller instances for position PID tuning
/// @details Chain that generates a trapezoidal velocity profile and uses
///          feedforward + PID feedback for pose control.
///          Chain: PoseErrorFilter -> ProfileFeedforwardController ->
///                 PosePIDController -> FeedforwardCombinerController ->
///                 SpeedPIDController -> TuningPoseReachedFilter

#pragma once

#include "app_conf.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerController.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerControllerIOKeys.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerControllerParameters.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "pose_error_filter/PoseErrorFilter.hpp"
#include "pose_error_filter/PoseErrorFilterIOKeys.hpp"
#include "pose_error_filter/PoseErrorFilterParameters.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilter.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilterIOKeys.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace angular_pose_tuning_chain {

// ============================================================================
// PIDs for angular_pose_tuning_chain (separate instances)
// ============================================================================

inline cogip::pid::PIDParameters
    angular_pose_tuning_pid_parameters(angular_pose_pid_kp, angular_pose_pid_ki,
                                       angular_pose_pid_kd, angular_pose_pid_integral_limit);
inline cogip::pid::PID angular_pose_tuning_pid(angular_pose_tuning_pid_parameters);

inline cogip::pid::PIDParameters
    angular_speed_tuning_pid_parameters(angular_speed_pid_kp, angular_speed_pid_ki,
                                        angular_speed_pid_kd, angular_speed_pid_integral_limit);
inline cogip::pid::PID angular_speed_tuning_pid(angular_speed_tuning_pid_parameters);

inline cogip::motion_control::PosePIDControllerParameters
    pose_pid_controller_parameters(&angular_pose_tuning_pid);
inline cogip::motion_control::SpeedPIDControllerParameters
    speed_pid_controller_parameters(&angular_speed_tuning_pid);

// ============================================================================
// PoseErrorFilter for computing angle difference to target
// ============================================================================

inline cogip::motion_control::PoseErrorFilterIOKeys pose_error_filter_io_keys = {
    .target_x = "", // Not used in ANGULAR mode
    .target_y = "", // Not used in ANGULAR mode
    .target_O = "target_pose_O",
    .current_x = "", // Not used in ANGULAR mode
    .current_y = "", // Not used in ANGULAR mode
    .current_O = "current_pose_O",
    .pose_error = "angular_pose_error",
    .recompute = "angular_pose_recompute_profile"};

inline cogip::motion_control::PoseErrorFilterParameters pose_error_filter_parameters(
    cogip::motion_control::PoseErrorFilterMode::ANGULAR,
    0.0f // pose_reached_threshold (not used, TuningPoseReachedFilter handles this)
);

inline cogip::motion_control::PoseErrorFilter pose_error_filter(pose_error_filter_io_keys,
                                                                pose_error_filter_parameters);

// ============================================================================
// ProfileFeedforwardController for velocity profile generation
// ============================================================================

/// IO keys: pose_error triggers the profile, feedforward_velocity is the output
inline cogip::motion_control::ProfileFeedforwardControllerIOKeys profile_feedforward_io_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .recompute_profile = "angular_pose_recompute_profile",
    .feedforward_velocity = "angular_feedforward_velocity",
    .tracking_error = "angular_pose_tracking_error", // For PosePID input
    .profile_complete = "angular_pose_profile_complete"};

inline cogip::motion_control::ProfileFeedforwardControllerParameters
    profile_feedforward_parameters(platform_max_speed_angular_deg_per_period, // max_speed
                                   platform_max_acc_angular_deg_per_period2,  // acceleration
                                   platform_max_dec_angular_deg_per_period2,  // deceleration
                                   true                                       // must_stop_at_end
    );

inline cogip::motion_control::ProfileFeedforwardController
    profile_feedforward_controller(profile_feedforward_io_keys, profile_feedforward_parameters);

// ============================================================================
// PosePIDController for position control (the PID we're tuning)
// ============================================================================

/// IO keys for angular pose PID controller
/// Takes tracking_error from profile, outputs pose_feedback correction
inline cogip::motion_control::PosePIDControllerIOKeys pose_pid_io_keys = {
    .position_error = "angular_pose_tracking_error", // From ProfileFeedforwardController
    .current_speed = "angular_current_speed",
    .target_speed = "",
    .disable_filter = "",
    .pose_reached = "",
    .speed_order = "angular_pose_feedback"}; // Output to Combiner

inline cogip::motion_control::PosePIDController pose_pid_controller(pose_pid_io_keys,
                                                                    pose_pid_controller_parameters);

// ============================================================================
// FeedforwardCombinerController (feedforward + pose feedback)
// ============================================================================

/// IO keys: combines feedforward velocity + pose PID feedback
/// speed_order = feedforward_velocity (setpoint for telemetry)
/// speed_command = feedforward_velocity + pose_feedback (input to SpeedPID)
inline cogip::motion_control::FeedforwardCombinerControllerIOKeys combiner_io_keys = {
    .feedforward_velocity = "angular_feedforward_velocity",
    .feedback_correction = "angular_pose_feedback",
    .speed_order = "angular_speed_order",       // Setpoint for telemetry
    .speed_command = "angular_speed_setpoint"}; // Input for SpeedPID (feedforward + pose feedback)

inline cogip::motion_control::FeedforwardCombinerControllerParameters combiner_parameters;

inline cogip::motion_control::FeedforwardCombinerController
    feedforward_combiner_controller(combiner_io_keys, combiner_parameters);

// ============================================================================
// SpeedPIDController (uses feedforward PID for tuning)
// ============================================================================

/// IO keys for speed PID: reads combined speed setpoint, outputs speed command
inline cogip::motion_control::SpeedPIDControllerIOKeys speed_pid_io_keys = {
    .speed_order = "angular_speed_setpoint", // From Combiner (feedforward + pose feedback)
    .current_speed = "angular_current_speed",
    .speed_command = "angular_speed_command"};

inline cogip::motion_control::SpeedPIDController speed_controller(speed_pid_io_keys,
                                                                  speed_pid_controller_parameters);

// ============================================================================
// TuningPoseReachedFilter for signaling pose_reached when profile completes
// ============================================================================

inline cogip::motion_control::TuningPoseReachedFilterIOKeys tuning_pose_reached_filter_io_keys = {
    .profile_complete = "angular_pose_profile_complete", .pose_reached = "pose_reached"};

inline cogip::motion_control::TuningPoseReachedFilter
    tuning_pose_reached_filter(tuning_pose_reached_filter_io_keys);

// ============================================================================
// Meta controller
// ============================================================================

inline cogip::motion_control::MetaController<> meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize angular pose tuning chain meta controller
cogip::motion_control::MetaController<>* init();

} // namespace angular_pose_tuning_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
