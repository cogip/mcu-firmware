// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Feedforward chain controller instances
/// @details Controllers that need separate instances for the feedforward chain
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
#include "feedforward_combiner_controller/FeedforwardCombinerController.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerControllerIOKeys.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerControllerParameters.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "parameter/Parameter.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "pose_straight_filter/PoseStraightFilterParameters.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardControllerIOKeys.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardControllerParameters.hpp"
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

namespace cogip {
namespace pf {
namespace motion_control {
namespace quadpid_feedforward_chain {

// ============================================================================
// Local PIDs (independent from QUADPID chain)
// ============================================================================

// PID parameters for feedforward chain
inline cogip::pid::PIDParameters feedforward_linear_pose_pid_parameters(
    feedforward_linear_pose_pid_kp, feedforward_linear_pose_pid_ki, feedforward_linear_pose_pid_kd,
    feedforward_linear_pose_pid_integral_limit);

inline cogip::pid::PIDParameters feedforward_linear_speed_pid_parameters(
    feedforward_linear_speed_pid_kp, feedforward_linear_speed_pid_ki,
    feedforward_linear_speed_pid_kd, feedforward_linear_speed_pid_integral_limit);

inline cogip::pid::PIDParameters feedforward_angular_pose_pid_parameters(
    feedforward_angular_pose_pid_kp, feedforward_angular_pose_pid_ki,
    feedforward_angular_pose_pid_kd, feedforward_angular_pose_pid_integral_limit);

inline cogip::pid::PIDParameters feedforward_angular_speed_pid_parameters(
    feedforward_angular_speed_pid_kp, feedforward_angular_speed_pid_ki,
    feedforward_angular_speed_pid_kd, feedforward_angular_speed_pid_integral_limit);

// PID instances
inline cogip::pid::PID feedforward_linear_pose_pid(feedforward_linear_pose_pid_parameters);
inline cogip::pid::PID feedforward_linear_speed_pid(feedforward_linear_speed_pid_parameters);
inline cogip::pid::PID feedforward_angular_pose_pid(feedforward_angular_pose_pid_parameters);
inline cogip::pid::PID feedforward_angular_speed_pid(feedforward_angular_speed_pid_parameters);

// ============================================================================
// PoseStraightFilter (separate instance with its own parameters)
// ============================================================================

/// Local parameters for PoseStraightFilter (independent from quadpid_chain)
inline cogip::motion_control::PoseStraightFilterParameters pose_straight_filter_parameters(
    angular_threshold, linear_threshold, angular_intermediate_threshold,
    platform_max_dec_angular_deg_per_period2, platform_max_dec_linear_mm_per_period2);

inline cogip::motion_control::PoseStraightFilter
    pose_straight_filter(cogip::motion_control::pose_straight_filter_io_keys_default,
                         pose_straight_filter_parameters);

// ============================================================================
// Pose loop meta controllers (ProfileFeedforward + ConditionalSwitch + Combiner)
// ============================================================================

// Linear dominant PosePIDController (strong gain for active tracking)
inline cogip::motion_control::PosePIDControllerParameters
    linear_feedforward_pose_controller_parameters{&feedforward_linear_pose_pid};

inline constexpr cogip::motion_control::PosePIDControllerIOKeys
    linear_feedforward_pose_controller_keys = {.position_error = "linear_tracking_error",
                                               .current_speed = "linear_current_speed",
                                               .target_speed = "dummy_target_speed",
                                               .disable_filter = "dummy_disable",
                                               .pose_reached = "dummy_pose_reached",
                                               .speed_order = "linear_feedback_correction"};

inline cogip::motion_control::PosePIDController linear_feedforward_pose_controller{
    linear_feedforward_pose_controller_keys, linear_feedforward_pose_controller_parameters};

// ============================================================================
// ProfileFeedforwardController instances
// ============================================================================

/// Linear ProfileFeedforwardController IO keys
inline cogip::motion_control::ProfileFeedforwardControllerIOKeys
    linear_profile_feedforward_io_keys = {
        .pose_error = "linear_pose_error",
        .current_speed = "linear_current_speed",
        .recompute_profile = "linear_recompute_profile",
        .feedforward_velocity = "linear_feedforward_velocity",
        .tracking_error = "linear_tracking_error",
        .profile_complete = ""}; // Not used, PoseStraightFilter handles pose_reached

/// Linear ProfileFeedforwardController parameters
inline cogip::motion_control::ProfileFeedforwardControllerParameters
    linear_profile_feedforward_parameters(platform_max_speed_linear_mm_per_period, // max_speed
                                          platform_max_acc_linear_mm_per_period2,  // acceleration
                                          platform_max_dec_linear_mm_per_period2,  // deceleration
                                          true, // must_stop_at_end
                                          1     // period_increment
    );

/// Linear ProfileFeedforwardController
inline cogip::motion_control::ProfileFeedforwardController
    linear_profile_feedforward_controller(linear_profile_feedforward_io_keys,
                                          linear_profile_feedforward_parameters);

/// Angular ProfileFeedforwardController IO keys
inline cogip::motion_control::ProfileFeedforwardControllerIOKeys
    angular_profile_feedforward_io_keys = {
        .pose_error = "angular_pose_error",
        .current_speed = "angular_current_speed",
        .recompute_profile = "angular_recompute_profile",
        .feedforward_velocity = "angular_feedforward_velocity",
        .tracking_error = "angular_tracking_error",
        .profile_complete = ""}; // Not used, PoseStraightFilter handles pose_reached

/// Angular ProfileFeedforwardController parameters
inline cogip::motion_control::ProfileFeedforwardControllerParameters
    angular_profile_feedforward_parameters(
        platform_max_speed_angular_deg_per_period, // max_speed
        platform_max_acc_angular_deg_per_period2,  // acceleration
        platform_max_dec_angular_deg_per_period2,  // deceleration (same as acc for angular)
        true,                                      // must_stop_at_end
        1                                          // period_increment
    );

/// Angular ProfileFeedforwardController
inline cogip::motion_control::ProfileFeedforwardController
    angular_profile_feedforward_controller(angular_profile_feedforward_io_keys,
                                           angular_profile_feedforward_parameters);

// ============================================================================
// FeedforwardCombinerController instances
// ============================================================================

/// Linear FeedforwardCombinerController IO keys (position tracker)
/// Outputs speed_order only (not speed_command) - speed combiner will produce speed_command
inline cogip::motion_control::FeedforwardCombinerControllerIOKeys
    linear_feedforward_combiner_io_keys = {.feedforward_velocity = "linear_feedforward_velocity",
                                           .feedback_correction = "linear_feedback_correction",
                                           .speed_order = "linear_speed_order",
                                           .speed_command = ""}; // Speed combiner will handle this

/// Linear FeedforwardCombinerController parameters
inline cogip::motion_control::FeedforwardCombinerControllerParameters
    linear_feedforward_combiner_parameters;

/// Linear FeedforwardCombinerController
inline cogip::motion_control::FeedforwardCombinerController
    linear_feedforward_combiner_controller(linear_feedforward_combiner_io_keys,
                                           linear_feedforward_combiner_parameters);

/// Angular FeedforwardCombinerController IO keys (position tracker)
/// Outputs speed_order only (not speed_command) - speed combiner will produce speed_command
inline cogip::motion_control::FeedforwardCombinerControllerIOKeys
    angular_feedforward_combiner_io_keys = {.feedforward_velocity = "angular_feedforward_velocity",
                                            .feedback_correction = "angular_feedback_correction",
                                            .speed_order = "angular_speed_order",
                                            .speed_command = ""}; // Speed combiner will handle this

/// Angular FeedforwardCombinerController parameters
inline cogip::motion_control::FeedforwardCombinerControllerParameters
    angular_feedforward_combiner_parameters;

/// Angular FeedforwardCombinerController
inline cogip::motion_control::FeedforwardCombinerController
    angular_feedforward_combiner_controller(angular_feedforward_combiner_io_keys,
                                            angular_feedforward_combiner_parameters);

// ============================================================================
// SpeedPIDController parameters (use local feedforward PIDs)
// ============================================================================

/// Linear SpeedPIDController parameters (points to feedforward_linear_speed_pid)
inline cogip::motion_control::SpeedPIDControllerParameters
    linear_feedforward_speed_controller_parameters(&feedforward_linear_speed_pid);

/// Angular SpeedPIDController parameters (points to feedforward_angular_speed_pid)
inline cogip::motion_control::SpeedPIDControllerParameters
    angular_feedforward_speed_controller_parameters(&feedforward_angular_speed_pid);

// ============================================================================
// SpeedPIDController instances
// ============================================================================

/// Linear SpeedPIDController (uses local feedforward PID)
inline cogip::motion_control::SpeedPIDController linear_feedforward_speed_controller(
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    linear_feedforward_speed_controller_parameters);

/// Angular SpeedPIDController (uses local feedforward PID)
inline cogip::motion_control::SpeedPIDController angular_feedforward_speed_controller(
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    angular_feedforward_speed_controller_parameters);

// ============================================================================
// Linear feedforward chain SpeedPIDController
// ============================================================================

// Linear feedforward chain SpeedPIDController (uses feedforward PIDs)
inline cogip::motion_control::SpeedPIDControllerParameters linear_feedforward_chain_speed_params{
    &feedforward_linear_speed_pid};

inline cogip::motion_control::SpeedPIDController linear_feedforward_chain_speed{
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    linear_feedforward_chain_speed_params};

// ============================================================================
// Linear feedforward chain MetaController
// ============================================================================

// Feedforward chain: PosePID → feedback_correction, Combiner → SpeedPID
inline cogip::motion_control::MetaController<> linear_feedforward_chain;

// ============================================================================
// Angular PosePIDController (tracking error correction)
// ============================================================================

// Angular dominant PosePIDController (strong gain for active tracking)
inline constexpr cogip::motion_control::PosePIDControllerIOKeys
    angular_feedforward_pose_controller_keys = {.position_error = "angular_tracking_error",
                                                .current_speed = "angular_current_speed",
                                                .target_speed = "dummy_target_speed",
                                                .disable_filter = "dummy_disable",
                                                .pose_reached = "dummy_pose_reached",
                                                .speed_order = "angular_feedback_correction"};

inline cogip::motion_control::PosePIDControllerParameters
    angular_feedforward_pose_controller_parameters{&feedforward_angular_pose_pid};

inline cogip::motion_control::PosePIDController angular_feedforward_pose_controller{
    angular_feedforward_pose_controller_keys, angular_feedforward_pose_controller_parameters};

// ============================================================================
// Angular feedforward chain SpeedPIDController
// ============================================================================

// Angular feedforward chain SpeedPIDController (uses feedforward PIDs)
inline cogip::motion_control::SpeedPIDControllerParameters angular_feedforward_chain_speed_params{
    &feedforward_angular_speed_pid};

inline cogip::motion_control::SpeedPIDController angular_feedforward_chain_speed{
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    angular_feedforward_chain_speed_params};

// ============================================================================
// Angular feedforward chain MetaController
// ============================================================================

// Feedforward chain: PosePID → feedback_correction, Combiner → SpeedPID
inline cogip::motion_control::MetaController<> angular_feedforward_chain;

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
    linear_anti_blocking_parameters(false, // disabled by default
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
// Telemetry controller
// ============================================================================

inline cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters;

inline cogip::motion_control::TelemetryController
    pose_telemetry_controller(cogip::motion_control::linear_telemetry_controller_io_keys_default,
                              telemetry_controller_parameters);

// ============================================================================
// QuadPIDMetaController for feedforward chain
// ============================================================================

inline cogip::motion_control::QuadPIDMetaController quadpid_feedforward_meta_controller;

// ============================================================================
// Chain initialization function
// ============================================================================

/// Initialize feedforward chain meta controller
cogip::motion_control::QuadPIDMetaController* init();

/// Reset feedforward chain state (stateful filters and PIDs)
inline void reset()
{
    // Reset feedforward PIDs (clear integral terms)
    feedforward_linear_speed_pid.reset();
    feedforward_angular_speed_pid.reset();
    feedforward_linear_pose_pid.reset();
    feedforward_angular_pose_pid.reset();

    // Note: PoseStraightFilter, ProfileFeedforwardController, DecelerationFilter,
    // and SpeedLimitFilter don't have reset() methods as they are either stateless
    // or reset automatically on new targets
}

} // namespace quadpid_feedforward_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
