// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Combined pose tuning chain (linear + angular) for position PID tuning
/// @details Chain that runs linear and angular pose control in parallel.
///          Uses trapezoidal velocity profiles for both axes.
///          All controller instances are dedicated to this chain to avoid
///          state sharing issues with other chains.

#pragma once

#include "app_conf.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_error_filter/PoseErrorFilter.hpp"
#include "pose_error_filter/PoseErrorFilterIOKeys.hpp"
#include "pose_error_filter/PoseErrorFilterParameters.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerIOKeys.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeys.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilter.hpp"
#include "tuning_pose_reached_filter/TuningPoseReachedFilterIOKeys.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

namespace pose_test_chain {

// ============================================================================
// PIDs for pose_test_chain (separate instances)
// ============================================================================

inline cogip::pid::PIDParameters
    pose_test_linear_pose_pid_parameters(linear_pose_pid_kp, linear_pose_pid_ki, linear_pose_pid_kd,
                                         linear_pose_pid_integral_limit);
inline cogip::pid::PID pose_test_linear_pose_pid(pose_test_linear_pose_pid_parameters);

inline cogip::pid::PIDParameters
    pose_test_linear_speed_pid_parameters(linear_speed_pid_kp, linear_speed_pid_ki,
                                          linear_speed_pid_kd, linear_speed_pid_integral_limit);
inline cogip::pid::PID pose_test_linear_speed_pid(pose_test_linear_speed_pid_parameters);

inline cogip::pid::PIDParameters
    pose_test_angular_pose_pid_parameters(angular_pose_pid_kp, angular_pose_pid_ki,
                                          angular_pose_pid_kd, angular_pose_pid_integral_limit);
inline cogip::pid::PID pose_test_angular_pose_pid(pose_test_angular_pose_pid_parameters);

inline cogip::pid::PIDParameters
    pose_test_angular_speed_pid_parameters(angular_speed_pid_kp, angular_speed_pid_ki,
                                           angular_speed_pid_kd, angular_speed_pid_integral_limit);
inline cogip::pid::PID pose_test_angular_speed_pid(pose_test_angular_speed_pid_parameters);

// ============================================================================
// LINEAR POSE LOOP - Dedicated instances
// TargetChangeDetector -> PoseErrorFilter -> ProfileTracker -> PosePID -> Combiner
// ============================================================================

// TargetChangeDetector for linear
inline cogip::motion_control::TargetChangeDetectorIOKeys linear_target_change_detector_io_keys = {
    .target_x = "target_pose_x",
    .target_y = "target_pose_y",
    .target_O = "",
    .new_target = "linear_pose_recompute_profile",
    .trigger_state = 0};

inline cogip::motion_control::TargetChangeDetectorParameters
    linear_target_change_detector_parameters;

inline cogip::motion_control::TargetChangeDetector
    linear_target_change_detector(linear_target_change_detector_io_keys,
                                  linear_target_change_detector_parameters);

// PoseErrorFilter for linear
inline cogip::motion_control::PoseErrorFilterIOKeys linear_pose_error_filter_io_keys = {
    .target_x = "target_pose_x",
    .target_y = "target_pose_y",
    .target_O = "",
    .current_x = "current_pose_x",
    .current_y = "current_pose_y",
    .current_O = "current_pose_O",
    .pose_error = "linear_pose_error",
    .new_target = "linear_pose_recompute_profile"};

inline cogip::motion_control::PoseErrorFilterParameters
    linear_pose_error_filter_parameters(cogip::motion_control::PoseErrorFilterMode::LINEAR, 0.0f);

inline cogip::motion_control::PoseErrorFilter
    linear_pose_error_filter(linear_pose_error_filter_io_keys, linear_pose_error_filter_parameters);

// ProfileTrackerController for linear
inline cogip::motion_control::ProfileTrackerControllerIOKeys linear_profile_tracker_io_keys = {
    .pose_error = "linear_pose_error",
    .current_speed = "linear_current_speed",
    .recompute_profile = "linear_pose_recompute_profile",
    .tracker_velocity = "linear_tracker_velocity",
    .tracking_error = "linear_pose_tracking_error",
    .profile_complete = "linear_pose_profile_complete",
    .target_speed = ""}; // Not used in pose test mode

inline cogip::motion_control::ProfileTrackerControllerParameters
    linear_profile_tracker_parameters(platform_max_speed_linear_mm_per_period,
                                      platform_max_acc_linear_mm_per_period2,
                                      platform_max_dec_linear_mm_per_period2, true);

inline cogip::motion_control::ProfileTrackerController
    linear_profile_tracker_controller(linear_profile_tracker_io_keys,
                                      linear_profile_tracker_parameters);

// TrackerCombinerController for linear
inline cogip::motion_control::TrackerCombinerControllerIOKeys linear_combiner_io_keys = {
    .tracker_velocity = "linear_tracker_velocity",
    .feedback_correction = "linear_pose_feedback",
    .speed_order = "linear_speed_order",
    .speed_command = "linear_speed_setpoint"};

inline cogip::motion_control::TrackerCombinerControllerParameters linear_combiner_parameters;

inline cogip::motion_control::TrackerCombinerController
    linear_tracker_combiner_controller(linear_combiner_io_keys, linear_combiner_parameters);

// SpeedPIDController for linear
inline cogip::motion_control::SpeedPIDControllerIOKeys linear_speed_pid_io_keys = {
    .speed_order = "linear_speed_setpoint",
    .current_speed = "linear_current_speed",
    .speed_command = "linear_speed_command"};

inline cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_controller_parameters(&pose_test_linear_speed_pid);

inline cogip::motion_control::SpeedPIDController
    linear_speed_controller(linear_speed_pid_io_keys, linear_speed_controller_parameters);

// ============================================================================
// ANGULAR POSE LOOP - Dedicated instances
// TargetChangeDetector -> PoseErrorFilter -> PosePID (simplified)
// ============================================================================

// TargetChangeDetector for angular
inline cogip::motion_control::TargetChangeDetectorIOKeys angular_target_change_detector_io_keys = {
    .target_x = "",
    .target_y = "",
    .target_O = "target_pose_O",
    .new_target = "angular_pose_recompute_profile",
    .trigger_state = 0};

inline cogip::motion_control::TargetChangeDetectorParameters
    angular_target_change_detector_parameters;

inline cogip::motion_control::TargetChangeDetector
    angular_target_change_detector(angular_target_change_detector_io_keys,
                                   angular_target_change_detector_parameters);

// PoseErrorFilter for angular
inline cogip::motion_control::PoseErrorFilterIOKeys angular_pose_error_filter_io_keys = {
    .target_x = "",
    .target_y = "",
    .target_O = "target_pose_O",
    .current_x = "",
    .current_y = "",
    .current_O = "current_pose_O",
    .pose_error = "angular_pose_error",
    .new_target = "angular_pose_recompute_profile"};

inline cogip::motion_control::PoseErrorFilterParameters
    angular_pose_error_filter_parameters(cogip::motion_control::PoseErrorFilterMode::ANGULAR, 0.0f);

inline cogip::motion_control::PoseErrorFilter
    angular_pose_error_filter(angular_pose_error_filter_io_keys,
                              angular_pose_error_filter_parameters);

// ProfileTrackerController for angular
inline cogip::motion_control::ProfileTrackerControllerIOKeys angular_profile_tracker_io_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .recompute_profile = "angular_pose_recompute_profile",
    .tracker_velocity = "angular_tracker_velocity",
    .tracking_error = "angular_pose_tracking_error",
    .profile_complete = "angular_pose_profile_complete",
    .target_speed = ""}; // Not used in pose test mode

inline cogip::motion_control::ProfileTrackerControllerParameters
    angular_profile_tracker_parameters(platform_max_speed_angular_deg_per_period,
                                       platform_max_acc_angular_deg_per_period2,
                                       platform_max_dec_angular_deg_per_period2, true);

inline cogip::motion_control::ProfileTrackerController
    angular_profile_tracker_controller(angular_profile_tracker_io_keys,
                                       angular_profile_tracker_parameters);

// TrackerCombinerController for angular
inline cogip::motion_control::TrackerCombinerControllerIOKeys angular_combiner_io_keys = {
    .tracker_velocity = "angular_tracker_velocity",
    .feedback_correction = "angular_pose_feedback",
    .speed_order = "angular_speed_order",
    .speed_command = "angular_speed_setpoint"};

inline cogip::motion_control::TrackerCombinerControllerParameters angular_combiner_parameters;

inline cogip::motion_control::TrackerCombinerController
    angular_tracker_combiner_controller(angular_combiner_io_keys, angular_combiner_parameters);

// SpeedPIDController for angular
inline cogip::motion_control::SpeedPIDControllerIOKeys angular_speed_pid_io_keys = {
    .speed_order = "angular_speed_setpoint",
    .current_speed = "angular_current_speed",
    .speed_command = "angular_speed_command"};

inline cogip::motion_control::SpeedPIDControllerParameters
    angular_speed_controller_parameters(&pose_test_angular_speed_pid);

inline cogip::motion_control::SpeedPIDController
    angular_speed_controller(angular_speed_pid_io_keys, angular_speed_controller_parameters);

// ============================================================================
// Meta controllers
// ============================================================================

inline cogip::motion_control::MetaController<5> linear_pose_loop_meta_controller;
inline cogip::motion_control::MetaController<3>
    angular_pose_loop_meta_controller; // Simplified: TargetChangeDetector + PoseErrorFilter +
                                       // PosePID
inline cogip::motion_control::PolarParallelMetaController pose_loop_polar_parallel_meta_controller;

inline cogip::motion_control::MetaController<1> linear_speed_meta_controller;
inline cogip::motion_control::MetaController<1> angular_speed_meta_controller;
inline cogip::motion_control::PolarParallelMetaController speed_loop_polar_parallel_meta_controller;

// ============================================================================
// TuningPoseReachedFilter
// ============================================================================

inline cogip::motion_control::TuningPoseReachedFilterIOKeys tuning_pose_reached_filter_io_keys = {
    .profile_complete = "linear_pose_profile_complete", .pose_reached = "pose_reached"};

inline cogip::motion_control::TuningPoseReachedFilter
    tuning_pose_reached_filter(tuning_pose_reached_filter_io_keys);

// ============================================================================
// Meta controller
// ============================================================================

inline cogip::motion_control::MetaController<> meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize pose test chain meta controller
cogip::motion_control::MetaController<>* init();

} // namespace pose_test_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
