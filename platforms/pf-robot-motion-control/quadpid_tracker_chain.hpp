// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Tracker chain controller instances
/// @details Controllers that need separate instances for the tracker chain
///          (cannot be shared with QuadPID chain due to meta-controller ownership).

#pragma once

#include "acceleration_filter/AccelerationFilter.hpp"
#include "acceleration_filter/AccelerationFilterIOKeys.hpp"
#include "acceleration_filter/AccelerationFilterParameters.hpp"
#include "anti_blocking_controller/AntiBlockingController.hpp"
#include "anti_blocking_controller/AntiBlockingControllerParameters.hpp"
#include "app_conf.hpp"
#include "conditional_switch_meta_controller/ConditionalSwitchMetaController.hpp"
#include "deceleration_filter/DecelerationFilter.hpp"
#include "deceleration_filter/DecelerationFilterIOKeys.hpp"
#include "deceleration_filter/DecelerationFilterParameters.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "parameter/Parameter.hpp"
#include "path_manager_filter/PathManagerFilter.hpp"
#include "path_manager_filter/PathManagerFilterIOKeys.hpp"
#include "path_manager_filter/PathManagerFilterParameters.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "pose_straight_filter/PoseStraightFilterParameters.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerIOKeys.hpp"
#include "profile_tracker_controller/ProfileTrackerControllerParameters.hpp"
#include "quadpid_meta_controller/QuadPIDMetaController.hpp"
#include "speed_limit_filter/SpeedLimitFilter.hpp"
#include "speed_limit_filter/SpeedLimitFilterIOKeys.hpp"
#include "speed_limit_filter/SpeedLimitFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeysDefault.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerIOKeys.hpp"
#include "tracker_combiner_controller/TrackerCombinerControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace quadpid_tracker_chain {

// ============================================================================
// Local PIDs (independent from QUADPID chain)
// ============================================================================

// PID parameters for tracker chain
inline cogip::pid::PIDParameters
    tracker_linear_pose_pid_parameters(tracker_linear_pose_pid_kp, tracker_linear_pose_pid_ki,
                                       tracker_linear_pose_pid_kd,
                                       tracker_linear_pose_pid_integral_limit);

inline cogip::pid::PIDParameters
    tracker_linear_speed_pid_parameters(tracker_linear_speed_pid_kp, tracker_linear_speed_pid_ki,
                                        tracker_linear_speed_pid_kd,
                                        tracker_linear_speed_pid_integral_limit);

inline cogip::pid::PIDParameters
    tracker_angular_pose_pid_parameters(tracker_angular_pose_pid_kp, tracker_angular_pose_pid_ki,
                                        tracker_angular_pose_pid_kd,
                                        tracker_angular_pose_pid_integral_limit);

inline cogip::pid::PIDParameters
    tracker_angular_speed_pid_parameters(tracker_angular_speed_pid_kp, tracker_angular_speed_pid_ki,
                                         tracker_angular_speed_pid_kd,
                                         tracker_angular_speed_pid_integral_limit);

// PID instances
inline cogip::pid::PID tracker_linear_pose_pid(tracker_linear_pose_pid_parameters);
inline cogip::pid::PID tracker_linear_speed_pid(tracker_linear_speed_pid_parameters);
inline cogip::pid::PID tracker_angular_pose_pid(tracker_angular_pose_pid_parameters);
inline cogip::pid::PID tracker_angular_speed_pid(tracker_angular_speed_pid_parameters);

// ============================================================================
// PathManagerFilter
// ============================================================================

inline constexpr cogip::motion_control::PathManagerFilterIOKeys path_manager_filter_io_keys = {
    .pose_reached = "pose_reached",
    .target_pose_x = "target_pose_x",
    .target_pose_y = "target_pose_y",
    .target_pose_O = "target_pose_O",
    .new_target = "new_target",
    .path_complete = "path_complete",
    .path_index = "path_index",
    .bypass_final_orientation = "bypass_final_orientation",
    .motion_direction = "motion_direction",
    .is_intermediate = "is_intermediate"};

inline cogip::motion_control::PathManagerFilterParameters path_manager_filter_parameters;

inline cogip::motion_control::PathManagerFilter path_manager_filter(path_manager_filter_io_keys,
                                                                    path_manager_filter_parameters);

// ============================================================================
// PoseStraightFilter (separate instance with its own parameters)
// ============================================================================

/// Local parameters for PoseStraightFilter (independent from quadpid_chain)
inline cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters(
    angular_threshold, linear_threshold, angular_intermediate_threshold,
    platform_max_dec_angular_deg_per_period2, platform_max_dec_linear_mm_per_period2,
    false, // bypass_final_orientation
    true   // use_angle_continuity (required for ProfileTracker)
);

inline cogip::motion_control::PoseStraightFilter
    pose_straight_filter(cogip::motion_control::pose_straight_filter_io_keys_default,
                         pose_straight_filter_parameters);

// ============================================================================
// Pose loop meta controllers (ProfileTracker + ConditionalSwitch + Combiner)
// ============================================================================

// Linear dominant PosePIDController (strong gain for active tracking)
inline cogip::motion_control::PosePIDControllerParameters linear_tracker_pose_controller_parameters{
    &tracker_linear_pose_pid};

inline constexpr cogip::motion_control::PosePIDControllerIOKeys
    linear_tracker_pose_controller_keys = {.position_error = "linear_tracking_error",
                                           .current_speed = "linear_current_speed",
                                           .target_speed = "dummy_target_speed",
                                           .disable_filter = "dummy_disable",
                                           .pose_reached = "dummy_pose_reached",
                                           .speed_order = "linear_feedback_correction",
                                           .reset = "linear_pose_pid_reset"};

inline cogip::motion_control::PosePIDController linear_tracker_pose_controller{
    linear_tracker_pose_controller_keys, linear_tracker_pose_controller_parameters};

// ============================================================================
// ProfileTrackerController instances
// ============================================================================

/// Linear ProfileTrackerController IO keys
/// Note: current_speed uses speed_order (not measured speed) so the profile
/// continues from commanded velocity when regenerating mid-motion
inline cogip::motion_control::ProfileTrackerControllerIOKeys linear_profile_tracker_io_keys = {
    .pose_error = "linear_pose_error",
    .current_speed = "linear_speed_order",
    .recompute_profile = "linear_recompute_profile",
    .tracker_velocity = "linear_tracker_velocity",
    .tracking_error = "linear_tracking_error",
    .profile_complete = "", // Not used, PoseStraightFilter handles pose_reached
    .target_speed = "linear_target_speed"};

/// Linear ProfileTrackerController parameters
inline cogip::motion_control::ProfileTrackerControllerParameters
    linear_profile_tracker_parameters(platform_max_speed_linear_mm_per_period, // max_speed
                                      platform_max_acc_linear_mm_per_period2,  // acceleration
                                      platform_max_dec_linear_mm_per_period2,  // deceleration
                                      true,                                    // must_stop_at_end
                                      1                                        // period_increment
    );

/// Linear ProfileTrackerController
inline cogip::motion_control::ProfileTrackerController
    linear_profile_tracker_controller(linear_profile_tracker_io_keys,
                                      linear_profile_tracker_parameters);

/// Angular ProfileTrackerController IO keys
/// Note: current_speed uses speed_order (not measured speed) so the profile
/// continues from commanded velocity when regenerating mid-motion
inline cogip::motion_control::ProfileTrackerControllerIOKeys angular_profile_tracker_io_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_speed_order",
    .recompute_profile = "angular_recompute_profile",
    .tracker_velocity = "angular_tracker_velocity",
    .tracking_error = "angular_tracking_error",
    .profile_complete = "", // Not used, PoseStraightFilter handles pose_reached
    .target_speed = "angular_target_speed"};

/// Angular ProfileTrackerController parameters
inline cogip::motion_control::ProfileTrackerControllerParameters angular_profile_tracker_parameters(
    platform_max_speed_angular_deg_per_period, // max_speed
    platform_max_acc_angular_deg_per_period2,  // acceleration
    platform_max_dec_angular_deg_per_period2,  // deceleration (same as acc for angular)
    true,                                      // must_stop_at_end
    1                                          // period_increment
);

/// Angular ProfileTrackerController
inline cogip::motion_control::ProfileTrackerController
    angular_profile_tracker_controller(angular_profile_tracker_io_keys,
                                       angular_profile_tracker_parameters);

// ============================================================================
// TrackerCombinerController instances
// ============================================================================

/// Linear TrackerCombinerController IO keys (position tracker)
/// Outputs speed_order only (not speed_command) - speed combiner will produce speed_command
inline cogip::motion_control::TrackerCombinerControllerIOKeys linear_tracker_combiner_io_keys = {
    .tracker_velocity = "linear_tracker_velocity",
    .feedback_correction = "linear_feedback_correction",
    .speed_order = "linear_speed_order",
    .speed_command = ""}; // Speed combiner will handle this

/// Linear TrackerCombinerController parameters
inline cogip::motion_control::TrackerCombinerControllerParameters
    linear_tracker_combiner_parameters;

/// Linear TrackerCombinerController
inline cogip::motion_control::TrackerCombinerController
    linear_tracker_combiner_controller(linear_tracker_combiner_io_keys,
                                       linear_tracker_combiner_parameters);

/// Angular TrackerCombinerController IO keys (position tracker)
/// Outputs speed_order only (not speed_command) - speed combiner will produce speed_command
inline cogip::motion_control::TrackerCombinerControllerIOKeys angular_tracker_combiner_io_keys = {
    .tracker_velocity = "angular_tracker_velocity",
    .feedback_correction = "angular_feedback_correction",
    .speed_order = "angular_speed_order",
    .speed_command = ""}; // Speed combiner will handle this

/// Angular TrackerCombinerController parameters
inline cogip::motion_control::TrackerCombinerControllerParameters
    angular_tracker_combiner_parameters;

/// Angular TrackerCombinerController
inline cogip::motion_control::TrackerCombinerController
    angular_tracker_combiner_controller(angular_tracker_combiner_io_keys,
                                        angular_tracker_combiner_parameters);

// ============================================================================
// SpeedPIDController parameters (use local tracker PIDs)
// ============================================================================

/// Linear SpeedPIDController parameters (points to tracker_linear_speed_pid)
inline cogip::motion_control::SpeedPIDControllerParameters
    linear_tracker_speed_controller_parameters(&tracker_linear_speed_pid);

/// Angular SpeedPIDController parameters (points to tracker_angular_speed_pid)
inline cogip::motion_control::SpeedPIDControllerParameters
    angular_tracker_speed_controller_parameters(&tracker_angular_speed_pid);

// ============================================================================
// SpeedPIDController instances
// ============================================================================

/// Linear SpeedPIDController (uses local tracker PID)
inline cogip::motion_control::SpeedPIDController linear_tracker_speed_controller(
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    linear_tracker_speed_controller_parameters);

/// Angular SpeedPIDController (uses local tracker PID)
inline cogip::motion_control::SpeedPIDController angular_tracker_speed_controller(
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    angular_tracker_speed_controller_parameters);

// ============================================================================
// Linear tracker chain SpeedPIDController
// ============================================================================

// Linear tracker chain SpeedPIDController (uses tracker PIDs)
inline cogip::motion_control::SpeedPIDControllerParameters linear_tracker_chain_speed_params{
    &tracker_linear_speed_pid};

inline cogip::motion_control::SpeedPIDController linear_tracker_chain_speed{
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    linear_tracker_chain_speed_params};

// ============================================================================
// Linear tracker chain MetaController
// ============================================================================

// Tracker chain: PosePID → feedback_correction, Combiner → SpeedPID
inline cogip::motion_control::MetaController<> linear_tracker_chain;

// ============================================================================
// Angular PosePIDController (tracking error correction)
// ============================================================================

// Angular dominant PosePIDController (strong gain for active tracking)
inline constexpr cogip::motion_control::PosePIDControllerIOKeys
    angular_tracker_pose_controller_keys = {.position_error = "angular_tracking_error",
                                            .current_speed = "angular_current_speed",
                                            .target_speed = "dummy_target_speed",
                                            .disable_filter = "dummy_disable",
                                            .pose_reached = "dummy_pose_reached",
                                            .speed_order = "angular_feedback_correction",
                                            .reset = "angular_pose_pid_reset"};

inline cogip::motion_control::PosePIDControllerParameters
    angular_tracker_pose_controller_parameters{&tracker_angular_pose_pid};

inline cogip::motion_control::PosePIDController angular_tracker_pose_controller{
    angular_tracker_pose_controller_keys, angular_tracker_pose_controller_parameters};

// ============================================================================
// Angular tracker chain SpeedPIDController
// ============================================================================

// Angular tracker chain SpeedPIDController (uses tracker PIDs)
inline cogip::motion_control::SpeedPIDControllerParameters angular_tracker_chain_speed_params{
    &tracker_angular_speed_pid};

inline cogip::motion_control::SpeedPIDController angular_tracker_chain_speed{
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    angular_tracker_chain_speed_params};

// ============================================================================
// Angular tracker chain MetaController
// ============================================================================

// Tracker chain: PosePID → feedback_correction, Combiner → SpeedPID
inline cogip::motion_control::MetaController<> angular_tracker_chain;

inline cogip::motion_control::MetaController<> linear_pose_loop_meta_controller;
inline cogip::motion_control::MetaController<> angular_pose_loop_meta_controller;

// PolarParallel for pose loop (linear + angular in parallel)
inline cogip::motion_control::PolarParallelMetaController pose_loop_polar_parallel_meta_controller;

// PolarParallel for anti-blocking controllers (common to all configurations)
inline cogip::motion_control::PolarParallelMetaController speed_loop_polar_parallel_meta_controller;

// ============================================================================
// TargetChangeDetector (separate instance - cannot be shared between chains)
// ============================================================================

inline cogip::motion_control::TargetChangeDetectorIOKeys target_change_detector_io_keys = {
    .target_x = "target_pose_x",
    .target_y = "target_pose_y",
    .target_O = "target_pose_O",
    .new_target = "new_target",
    .trigger_state = 0};

inline cogip::motion_control::TargetChangeDetectorParameters target_change_detector_parameters;

inline cogip::motion_control::TargetChangeDetector
    target_change_detector(target_change_detector_io_keys, target_change_detector_parameters);

// ============================================================================
// Anti-blocking controllers
// ============================================================================

inline cogip::motion_control::AntiBlockingControllerIOKeys linear_anti_blocking_io_keys = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .speed_error = "linear_speed_error",
    .pose_reached = "pose_reached"};

inline cogip::motion_control::AntiBlockingControllerParameters
    linear_anti_blocking_parameters(true, // enabled
                                    platform_linear_anti_blocking_speed_threshold_mm_per_period,
                                    platform_linear_anti_blocking_error_threshold_mm_per_period,
                                    platform_linear_anti_blocking_blocked_cycles_nb_threshold);

inline cogip::motion_control::AntiBlockingController
    linear_anti_blocking_controller(linear_anti_blocking_io_keys, linear_anti_blocking_parameters);

inline cogip::motion_control::AntiBlockingControllerIOKeys angular_anti_blocking_io_keys = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .speed_error = "angular_speed_error",
    .pose_reached = "pose_reached"};

// Angular anti-blocking disabled by default (same as QUADPID chain)
inline cogip::motion_control::AntiBlockingControllerParameters
    angular_anti_blocking_parameters(false, // disabled by default
                                     platform_linear_anti_blocking_speed_threshold_mm_per_period,
                                     platform_linear_anti_blocking_error_threshold_mm_per_period,
                                     platform_linear_anti_blocking_blocked_cycles_nb_threshold);

inline cogip::motion_control::AntiBlockingController
    angular_anti_blocking_controller(angular_anti_blocking_io_keys,
                                     angular_anti_blocking_parameters);

// ============================================================================
// Speed limit filters (safety clamp at ratio × max)
// ============================================================================

inline cogip::motion_control::SpeedLimitFilterIOKeys linear_speed_limit_io_keys = {
    .target_speed = "linear_speed_order", .output_speed = ""};

inline cogip::motion_control::SpeedLimitFilterParameters
linear_speed_limit_parameters(platform_min_speed_linear_mm_per_period,
                              platform_max_speed_linear_mm_per_period* speed_clamp_ratio);

inline cogip::motion_control::SpeedLimitFilter
    linear_speed_limit_filter(linear_speed_limit_io_keys, linear_speed_limit_parameters);

inline cogip::motion_control::SpeedLimitFilterIOKeys angular_speed_limit_io_keys = {
    .target_speed = "angular_speed_order", .output_speed = ""};

inline cogip::motion_control::SpeedLimitFilterParameters
angular_speed_limit_parameters(platform_min_speed_angular_deg_per_period,
                               platform_max_speed_angular_deg_per_period* speed_clamp_ratio);

inline cogip::motion_control::SpeedLimitFilter
    angular_speed_limit_filter(angular_speed_limit_io_keys, angular_speed_limit_parameters);

// ============================================================================
// Acceleration filters (safety clamp at ratio × max)
// ============================================================================

inline cogip::motion_control::AccelerationFilterIOKeys linear_acceleration_io_keys = {
    .target_speed = "linear_speed_order"};

inline cogip::motion_control::AccelerationFilterParameters
linear_acceleration_parameters(platform_max_acc_linear_mm_per_period2* acceleration_clamp_ratio,
                               platform_min_speed_linear_mm_per_period);

inline cogip::motion_control::AccelerationFilter
    linear_acceleration_filter(linear_acceleration_io_keys, linear_acceleration_parameters);

inline cogip::motion_control::AccelerationFilterIOKeys angular_acceleration_io_keys = {
    .target_speed = "angular_speed_order"};

inline cogip::motion_control::AccelerationFilterParameters
angular_acceleration_parameters(platform_max_acc_angular_deg_per_period2* acceleration_clamp_ratio,
                                platform_min_speed_angular_deg_per_period);

inline cogip::motion_control::AccelerationFilter
    angular_acceleration_filter(angular_acceleration_io_keys, angular_acceleration_parameters);

// ============================================================================
// Telemetry controller
// ============================================================================

inline cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters;

inline cogip::motion_control::TelemetryController
    pose_telemetry_controller(cogip::motion_control::linear_telemetry_controller_io_keys_default,
                              telemetry_controller_parameters);

// ============================================================================
// QuadPIDMetaController for tracker chain
// ============================================================================

inline cogip::motion_control::QuadPIDMetaController quadpid_tracker_meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize tracker chain meta controller
cogip::motion_control::QuadPIDMetaController* init();

/// Reset tracker chain state (all controllers via cascade)
inline void reset()
{
    // Reset all controllers via meta controller cascade
    // This will call reset() on each controller, which resets:
    // - PoseStraightFilter: state machine, prev_target, angular error tracking
    // - ProfileTrackerController: profile invalidation, period counter
    // - PosePIDController: PID integral term
    // - SpeedPIDController: PID integral term
    quadpid_tracker_meta_controller.reset();
}

} // namespace quadpid_tracker_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
