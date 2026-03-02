// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Unified tracker speed tuning chain (linear + angular) for speed PID tuning
/// @details Chain for tuning the tracker's speed PIDs with trapezoidal velocity profiles.
///          Runs linear and angular loops in parallel via PolarParallelMetaController.
///          The handler writes target_speed + duration into IO.
///          Each axis: TargetChangeDetector -> ProfileTrackerController(speed_mode) ->
///                     SpeedPIDController -> TrackerCombinerController

#pragma once

#include "app_conf.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeys.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace tracker_speed_tuning_chain {

// ============================================================================
// LINEAR PID (uses tracker parameters for tuning)
// ============================================================================

inline cogip::pid::PIDParameters tracker_linear_speed_tuning_pid_parameters(
    tracker_linear_speed_pid_kp, tracker_linear_speed_pid_ki, tracker_linear_speed_pid_kd,
    tracker_linear_speed_pid_integral_limit);
inline cogip::pid::PID tracker_linear_speed_tuning_pid(tracker_linear_speed_tuning_pid_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    tracker_linear_speed_tuning_controller_parameters(&tracker_linear_speed_tuning_pid);

// ============================================================================
// ANGULAR PID (uses tracker parameters for tuning)
// ============================================================================

inline cogip::pid::PIDParameters tracker_angular_speed_tuning_pid_parameters(
    tracker_angular_speed_pid_kp, tracker_angular_speed_pid_ki, tracker_angular_speed_pid_kd,
    tracker_angular_speed_pid_integral_limit);
inline cogip::pid::PID
    tracker_angular_speed_tuning_pid(tracker_angular_speed_tuning_pid_parameters);

inline cogip::motion_control::SpeedPIDControllerParameters
    tracker_angular_speed_tuning_controller_parameters(&tracker_angular_speed_tuning_pid);

// ============================================================================
// LINEAR SPEED LOOP
// TargetChangeDetector -> ProfileTrackerController(speed_mode) -> SpeedPIDController ->
//     TrackerCombinerController
// ============================================================================

inline cogip::motion_control::TargetChangeDetectorIOKeys<2> linear_target_change_detector_io_keys =
    {.watched_keys = {"linear_target_speed", "timeout_duration_period"},
     .new_target = "linear_recompute_profile"};

inline cogip::motion_control::TargetChangeDetectorParameters
    linear_target_change_detector_parameters;

inline cogip::motion_control::TargetChangeDetector<2>
    linear_target_change_detector(linear_target_change_detector_io_keys,
                                  linear_target_change_detector_parameters);

inline cogip::motion_control::ProfileTrackerControllerIOKeys linear_profile_tracker_io_keys = {
    .pose_error = "",
    .current_speed = "linear_current_speed",
    .recompute_profile = "linear_recompute_profile",
    .tracker_velocity = "linear_tracker_velocity",
    .tracking_error = "",
    .profile_complete = "",
    .target_speed = "linear_target_speed",
    .duration_periods = "timeout_duration_period"};

inline cogip::motion_control::ProfileTrackerControllerParameters
    linear_profile_tracker_parameters(platform_max_speed_linear_mm_per_period,
                                      platform_max_acc_linear_mm_per_period2,
                                      platform_max_dec_linear_mm_per_period2,
                                      true,  // must_stop_at_end
                                      1,     // period_increment
                                      true); // speed_mode

inline cogip::motion_control::ProfileTrackerController
    linear_profile_tracker_controller(linear_profile_tracker_io_keys,
                                      linear_profile_tracker_parameters);

inline cogip::motion_control::SpeedPIDControllerIOKeys linear_speed_pid_io_keys = {
    .speed_order = "linear_tracker_velocity",
    .current_speed = "linear_current_speed",
    .speed_command = "linear_speed_feedback"};

inline cogip::motion_control::SpeedPIDController
    linear_speed_controller(linear_speed_pid_io_keys,
                            tracker_linear_speed_tuning_controller_parameters);

inline cogip::motion_control::TrackerCombinerControllerIOKeys linear_combiner_io_keys = {
    .tracker_velocity = "linear_tracker_velocity",
    .feedback_correction = "linear_speed_feedback",
    .speed_order = "linear_speed_order",
    .speed_command = "linear_speed_command"};

inline cogip::motion_control::TrackerCombinerControllerParameters linear_combiner_parameters;

inline cogip::motion_control::TrackerCombinerController
    linear_tracker_combiner_controller(linear_combiner_io_keys, linear_combiner_parameters);

// ============================================================================
// ANGULAR SPEED LOOP
// TargetChangeDetector -> ProfileTrackerController(speed_mode) -> SpeedPIDController ->
//     TrackerCombinerController
// ============================================================================

inline cogip::motion_control::TargetChangeDetectorIOKeys<2> angular_target_change_detector_io_keys =
    {.watched_keys = {"angular_target_speed", "timeout_duration_period"},
     .new_target = "angular_recompute_profile"};

inline cogip::motion_control::TargetChangeDetectorParameters
    angular_target_change_detector_parameters;

inline cogip::motion_control::TargetChangeDetector<2>
    angular_target_change_detector(angular_target_change_detector_io_keys,
                                   angular_target_change_detector_parameters);

inline cogip::motion_control::ProfileTrackerControllerIOKeys angular_profile_tracker_io_keys = {
    .pose_error = "",
    .current_speed = "angular_current_speed",
    .recompute_profile = "angular_recompute_profile",
    .tracker_velocity = "angular_tracker_velocity",
    .tracking_error = "",
    .profile_complete = "",
    .target_speed = "angular_target_speed",
    .duration_periods = "timeout_duration_period"};

inline cogip::motion_control::ProfileTrackerControllerParameters
    angular_profile_tracker_parameters(platform_max_speed_angular_deg_per_period,
                                       platform_max_acc_angular_deg_per_period2,
                                       platform_max_dec_angular_deg_per_period2,
                                       true,  // must_stop_at_end
                                       1,     // period_increment
                                       true); // speed_mode

inline cogip::motion_control::ProfileTrackerController
    angular_profile_tracker_controller(angular_profile_tracker_io_keys,
                                       angular_profile_tracker_parameters);

inline cogip::motion_control::SpeedPIDControllerIOKeys angular_speed_pid_io_keys = {
    .speed_order = "angular_tracker_velocity",
    .current_speed = "angular_current_speed",
    .speed_command = "angular_speed_feedback"};

inline cogip::motion_control::SpeedPIDController
    angular_speed_controller(angular_speed_pid_io_keys,
                             tracker_angular_speed_tuning_controller_parameters);

inline cogip::motion_control::TrackerCombinerControllerIOKeys angular_combiner_io_keys = {
    .tracker_velocity = "angular_tracker_velocity",
    .feedback_correction = "angular_speed_feedback",
    .speed_order = "angular_speed_order",
    .speed_command = "angular_speed_command"};

inline cogip::motion_control::TrackerCombinerControllerParameters angular_combiner_parameters;

inline cogip::motion_control::TrackerCombinerController
    angular_tracker_combiner_controller(angular_combiner_io_keys, angular_combiner_parameters);

// ============================================================================
// Meta controllers
// ============================================================================

inline cogip::motion_control::MetaController<4> linear_meta_controller;
inline cogip::motion_control::MetaController<4> angular_meta_controller;
inline cogip::motion_control::PolarParallelMetaController polar_parallel_meta_controller;
inline cogip::motion_control::MetaController<> meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize tracker speed tuning chain meta controller
cogip::motion_control::MetaController<>* init();

} // namespace tracker_speed_tuning_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
