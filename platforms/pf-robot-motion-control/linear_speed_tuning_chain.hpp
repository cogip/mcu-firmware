// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Linear speed chain controller instances for speed PID tuning
/// @details Chain that generates a trapezoidal velocity profile and uses
///          tracker + PID feedback for speed control.
///          Chain: PoseErrorFilter -> ProfileTrackerController ->
///                 SpeedPIDController -> TrackerCombinerController ->
///                 TuningPoseReachedFilter

#pragma once

#include "app_conf.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "pose_error_filter/PoseErrorFilter.hpp"
#include "pose_error_filter/PoseErrorFilterIOKeys.hpp"
#include "pose_error_filter/PoseErrorFilterParameters.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeys.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilter.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilterIOKeys.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace linear_speed_tuning_chain {

// ============================================================================
// PID for linear_speed_tuning_chain (separate instance)
// ============================================================================

inline cogip::pid::PIDParameters
    linear_speed_tuning_pid_parameters(linear_speed_pid_kp, linear_speed_pid_ki,
                                       linear_speed_pid_kd, linear_speed_pid_integral_limit);
inline cogip::pid::PID linear_speed_tuning_pid(linear_speed_tuning_pid_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_tuning_controller_parameters(&linear_speed_tuning_pid);

// ============================================================================
// TargetChangeDetector for detecting target changes
// ============================================================================

inline cogip::motion_control::TargetChangeDetectorIOKeys target_change_detector_io_keys = {
    .target_x = "target_pose_x",
    .target_y = "target_pose_y",
    .target_O = "",
    .new_target = "linear_speed_recompute_profile",
    .trigger_state = 0};

inline cogip::motion_control::TargetChangeDetectorParameters target_change_detector_parameters;

inline cogip::motion_control::TargetChangeDetector
    target_change_detector(target_change_detector_io_keys, target_change_detector_parameters);

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
    .new_target = "linear_speed_recompute_profile"};

inline cogip::motion_control::PoseErrorFilterParameters pose_error_filter_parameters(
    cogip::motion_control::PoseErrorFilterMode::LINEAR,
    0.0f // pose_reached_threshold (not used, TuningPoseReachedFilter handles this)
);

inline cogip::motion_control::PoseErrorFilter pose_error_filter(pose_error_filter_io_keys,
                                                                pose_error_filter_parameters);

// ============================================================================
// ProfileTrackerController for velocity profile generation
// ============================================================================

/// IO keys: pose_error triggers the profile, tracker_velocity is the output
inline cogip::motion_control::ProfileTrackerControllerIOKeys profile_tracker_io_keys = {
    .pose_error = "linear_pose_error",
    .current_speed = "linear_current_speed",
    .recompute_profile = "linear_speed_recompute_profile",
    .tracker_velocity = "linear_tracker_velocity", // To Combiner
    .tracking_error = "linear_speed_tracking_error",
    .profile_complete = "linear_speed_profile_complete",
    .target_speed = ""}; // Not used in tuning mode

inline cogip::motion_control::ProfileTrackerControllerParameters
    profile_tracker_parameters(platform_max_speed_linear_mm_per_period, // max_speed
                               platform_max_acc_linear_mm_per_period2,  // acceleration
                               platform_max_dec_linear_mm_per_period2,  // deceleration
                               true                                     // must_stop_at_end
    );

inline cogip::motion_control::ProfileTrackerController
    profile_tracker_controller(profile_tracker_io_keys, profile_tracker_parameters);

// ============================================================================
// SpeedPIDController (computes feedback correction)
// ============================================================================

/// IO keys for speed PID: reads tracker velocity, outputs feedback correction
inline cogip::motion_control::SpeedPIDControllerIOKeys speed_pid_io_keys = {
    .speed_order = "linear_tracker_velocity", // Input from ProfileTrackerController
    .current_speed = "linear_current_speed",
    .speed_command = "linear_speed_feedback"}; // Output to Combiner (feedback correction)

inline cogip::motion_control::SpeedPIDController
    speed_controller(speed_pid_io_keys, linear_speed_tuning_controller_parameters);

// ============================================================================
// TrackerCombinerController (tracker + feedback)
// ============================================================================

/// IO keys: combines tracker velocity + PID feedback correction
/// speed_order = tracker_velocity (setpoint from profile)
/// speed_command = tracker_velocity + feedback (sent to motors)
inline cogip::motion_control::TrackerCombinerControllerIOKeys combiner_io_keys = {
    .tracker_velocity = "linear_tracker_velocity",
    .feedback_correction = "linear_speed_feedback",
    .speed_order = "linear_speed_order",      // Setpoint for telemetry
    .speed_command = "linear_speed_command"}; // Output for motors (tracker + feedback)

inline cogip::motion_control::TrackerCombinerControllerParameters combiner_parameters;

inline cogip::motion_control::TrackerCombinerController
    tracker_combiner_controller(combiner_io_keys, combiner_parameters);

// ============================================================================
// TuningPoseReachedFilter for signaling pose_reached when profile completes
// ============================================================================

inline cogip::motion_control::TuningPoseReachedFilterIOKeys tuning_pose_reached_filter_io_keys = {
    .profile_complete = "linear_speed_profile_complete", .pose_reached = "pose_reached"};

inline cogip::motion_control::TuningPoseReachedFilter
    tuning_pose_reached_filter(tuning_pose_reached_filter_io_keys);

// ============================================================================
// Meta controller
// ============================================================================

inline cogip::motion_control::MetaController<> meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize linear speed tuning chain meta controller
cogip::motion_control::MetaController<>* init();

} // namespace linear_speed_tuning_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
