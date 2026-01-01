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
#include "motion_control_common/ThrottledController.hpp"
#include "parameter/Parameter.hpp"
#include "pid/PID.hpp"
#include "polar_parallel_meta_controller/PolarParallelMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardControllerIOKeys.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardControllerParameters.hpp"
#include "quadpid_chain.hpp"
#include "speed_limit_filter/SpeedLimitFilter.hpp"
#include "speed_limit_filter/SpeedLimitFilterIOKeys.hpp"
#include "speed_limit_filter/SpeedLimitFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace quadpid_feedforward_chain {

// ============================================================================
// Local PIDs (independent from QUADPID chain)
// ============================================================================

// PID parameters for feedforward chain
extern cogip::pid::PID feedforward_linear_pose_pid;
extern cogip::pid::PID feedforward_linear_speed_pid;
extern cogip::pid::PID feedforward_angular_pose_pid;
extern cogip::pid::PID feedforward_angular_speed_pid;

// ============================================================================
// PoseStraightFilter (separate instance - cannot be shared between chains)
// Uses same parameters as quadpid_chain to share configuration
// ============================================================================

inline cogip::motion_control::PoseStraightFilter
    pose_straight_filter(cogip::motion_control::pose_straight_filter_io_keys_default,
                         quadpid_chain::pose_straight_filter_parameters);

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
extern cogip::motion_control::ProfileFeedforwardControllerIOKeys linear_profile_feedforward_io_keys;

/// Linear ProfileFeedforwardController parameters
extern cogip::motion_control::ProfileFeedforwardControllerParameters
    linear_profile_feedforward_parameters;

/// Linear ProfileFeedforwardController
extern cogip::motion_control::ProfileFeedforwardController linear_profile_feedforward_controller;

/// Angular ProfileFeedforwardController IO keys
extern cogip::motion_control::ProfileFeedforwardControllerIOKeys
    angular_profile_feedforward_io_keys;

/// Angular ProfileFeedforwardController parameters
extern cogip::motion_control::ProfileFeedforwardControllerParameters
    angular_profile_feedforward_parameters;

/// Angular ProfileFeedforwardController
extern cogip::motion_control::ProfileFeedforwardController angular_profile_feedforward_controller;

// ============================================================================
// FeedforwardCombinerController instances
// ============================================================================

/// Linear FeedforwardCombinerController IO keys
extern cogip::motion_control::FeedforwardCombinerControllerIOKeys
    linear_feedforward_combiner_io_keys;

/// Linear FeedforwardCombinerController parameters
extern cogip::motion_control::FeedforwardCombinerControllerParameters
    linear_feedforward_combiner_parameters;

/// Linear FeedforwardCombinerController
extern cogip::motion_control::FeedforwardCombinerController linear_feedforward_combiner_controller;

/// Angular FeedforwardCombinerController IO keys
extern cogip::motion_control::FeedforwardCombinerControllerIOKeys
    angular_feedforward_combiner_io_keys;

/// Angular FeedforwardCombinerController parameters
extern cogip::motion_control::FeedforwardCombinerControllerParameters
    angular_feedforward_combiner_parameters;

/// Angular FeedforwardCombinerController
extern cogip::motion_control::FeedforwardCombinerController angular_feedforward_combiner_controller;

// ============================================================================
// SpeedPIDController instances
// ============================================================================

/// Linear SpeedPIDController
extern cogip::motion_control::SpeedPIDController linear_feedforward_speed_controller;

/// Angular SpeedPIDController
extern cogip::motion_control::SpeedPIDController angular_feedforward_speed_controller;

// ============================================================================
// Angular dominant PosePIDController (strong gain for active tracking)
// ============================================================================

// Linear position corrector PID (uses corrector-specific coefficients from robot*_conf.hpp)
// Separate from QUADPID chain to allow independent tuning
inline cogip::pid::PIDParameters linear_position_corrector_pid_params{
    corrector_linear_pose_pid_kp, corrector_linear_pose_pid_ki, corrector_linear_pose_pid_kd,
    corrector_linear_pose_pid_integral_limit};
inline cogip::pid::PID linear_position_corrector_pid{linear_position_corrector_pid_params};

// Linear position corrector controller (uses corrector PID for position correction)
// When profile is invalidated, outputs directly to speed_order
// Uses pose_error (real distance to target) not tracking_error (profile following error)
inline cogip::motion_control::PosePIDControllerParameters linear_position_corrector_params{
    &linear_position_corrector_pid};

inline constexpr cogip::motion_control::PosePIDControllerIOKeys linear_position_corrector_keys = {
    .position_error = "linear_pose_error", // Real error to target, not tracking error
    .current_speed = "linear_current_speed",
    .target_speed = "dummy_target_speed",
    .disable_filter = "dummy_disable",
    .pose_reached = "dummy_pose_reached",
    .speed_order = "linear_speed_order" // Output directly to speed_order for direct mode
};

inline cogip::motion_control::PosePIDController linear_position_corrector{
    linear_position_corrector_keys, linear_position_corrector_params};

// Linear corrector AccelerationFilter (limits speed increase rate)
inline cogip::motion_control::AccelerationFilterIOKeys linear_corrector_accel_keys = {
    .target_speed = "linear_speed_order", .output_speed = "linear_speed_order"};

inline cogip::motion_control::AccelerationFilterParameters linear_corrector_accel_params{
    platform_max_acc_linear_mm_per_period2,
    platform_min_speed_linear_mm_per_period // min_speed: guaranteed startup speed
};

inline cogip::motion_control::AccelerationFilter linear_corrector_accel{
    linear_corrector_accel_keys, linear_corrector_accel_params, "lin"};

// Linear corrector DecelerationFilter (limits speed based on braking distance)
inline cogip::motion_control::DecelerationFilterIOKeys linear_corrector_decel_keys = {
    .pose_error = "linear_pose_error",
    .current_speed = "linear_current_speed",
    .target_speed = "linear_speed_order",
    .output_speed = "linear_speed_order"};

inline cogip::motion_control::DecelerationFilterParameters linear_corrector_decel_params{
    platform_max_dec_linear_mm_per_period2};

inline cogip::motion_control::DecelerationFilter linear_corrector_decel{
    linear_corrector_decel_keys, linear_corrector_decel_params};

// Linear corrector SpeedLimitFilter (clamps min/max speed)
inline cogip::motion_control::SpeedLimitFilterIOKeys linear_corrector_speed_limit_keys = {
    .target_speed = "linear_speed_order", .output_speed = "linear_speed_order"};

inline cogip::motion_control::SpeedLimitFilterParameters linear_corrector_speed_limit_params{
    platform_min_speed_linear_mm_per_period, platform_max_speed_linear_mm_per_period};

inline cogip::motion_control::SpeedLimitFilter linear_corrector_speed_limit{
    linear_corrector_speed_limit_keys, linear_corrector_speed_limit_params, "lin"};

// MetaController for feedforward mode: PosePID → feedback_correction, then Combiner → speed_order
// This chain is used when invalidate_profile=false (feedforward active)
inline cogip::motion_control::MetaController<2> linear_feedforward_chain;

// MetaController for direct mode: PosePID → AccelerationFilter → DecelerationFilter → speed_order
// This chain is used when invalidate_profile=true (no feedforward)
inline cogip::motion_control::MetaController<4> linear_direct_chain;

// Linear pose switch: switches between feedforward chain and direct chain
// Condition: linear_invalidate_profile
//   - true  → direct mode (position corrector outputs to speed_order)
//   - false → feedforward mode (tracker + combiner)
inline cogip::motion_control::ConditionalSwitchMetaController linear_pose_switch{
    "linear_invalidate_profile", &linear_direct_chain, &linear_feedforward_chain};

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

// Angular position corrector PID (uses corrector-specific coefficients from robot*_conf.hpp)
// Separate from QUADPID chain to allow independent tuning
inline cogip::pid::PIDParameters angular_position_corrector_pid_params{
    corrector_angular_pose_pid_kp, corrector_angular_pose_pid_ki, corrector_angular_pose_pid_kd,
    corrector_angular_pose_pid_integral_limit};
inline cogip::pid::PID angular_position_corrector_pid{angular_position_corrector_pid_params};

// Angular position corrector controller
// When profile is invalidated, outputs to SpeedLimitFilter
// Uses pose_error (real angle to target) not tracking_error (profile following error)
inline constexpr cogip::motion_control::PosePIDControllerIOKeys angular_position_corrector_keys = {
    .position_error = "angular_pose_error", // Real error to target, not tracking error
    .current_speed = "angular_current_speed",
    .target_speed = "dummy_target_speed",
    .disable_filter = "dummy_disable",
    .pose_reached = "dummy_pose_reached",
    .speed_order = "angular_speed_order" // Output directly to speed_order for direct mode
};

inline cogip::motion_control::PosePIDControllerParameters angular_position_corrector_params{
    &angular_position_corrector_pid};

inline cogip::motion_control::PosePIDController angular_position_corrector{
    angular_position_corrector_keys, angular_position_corrector_params};

// Angular corrector AccelerationFilter (limits speed increase rate)
inline cogip::motion_control::AccelerationFilterIOKeys angular_corrector_accel_keys = {
    .target_speed = "angular_speed_order", .output_speed = "angular_speed_order"};

inline cogip::motion_control::AccelerationFilterParameters angular_corrector_accel_params{
    platform_max_acc_angular_deg_per_period2,
    platform_min_speed_angular_deg_per_period // min_speed: guaranteed startup speed
};

inline cogip::motion_control::AccelerationFilter angular_corrector_accel{
    angular_corrector_accel_keys, angular_corrector_accel_params, "ang"};

// Angular corrector DecelerationFilter (limits speed based on braking distance)
inline cogip::motion_control::DecelerationFilterIOKeys angular_corrector_decel_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .target_speed = "angular_speed_order",
    .output_speed = "angular_speed_order"};

inline cogip::motion_control::DecelerationFilterParameters angular_corrector_decel_params{
    platform_max_dec_angular_deg_per_period2};

inline cogip::motion_control::DecelerationFilter angular_corrector_decel{
    angular_corrector_decel_keys, angular_corrector_decel_params};

// Angular corrector SpeedLimitFilter (clamps min/max angular speed)
inline cogip::motion_control::SpeedLimitFilterIOKeys angular_corrector_speed_limit_keys = {
    .target_speed = "angular_speed_order", .output_speed = "angular_speed_order"};

inline cogip::motion_control::SpeedLimitFilterParameters angular_corrector_speed_limit_params{
    platform_min_speed_angular_deg_per_period, platform_max_speed_angular_deg_per_period};

inline cogip::motion_control::SpeedLimitFilter angular_corrector_speed_limit{
    angular_corrector_speed_limit_keys, angular_corrector_speed_limit_params, "ang"};

// MetaController for feedforward mode: PosePID → feedback_correction, then Combiner → speed_order
// This chain is used when invalidate_profile=false (feedforward active)
inline cogip::motion_control::MetaController<2> angular_feedforward_chain;

// MetaController for direct mode: PosePID → AccelerationFilter → DecelerationFilter → speed_order
// This chain is used when invalidate_profile=true (no feedforward)
inline cogip::motion_control::MetaController<4> angular_direct_chain;

// Angular pose switch: switches between feedforward chain and direct chain
// Condition: angular_invalidate_profile
//   - true  → direct mode (position corrector outputs to speed_order)
//   - false → feedforward mode (tracker + combiner)
inline cogip::motion_control::ConditionalSwitchMetaController angular_pose_switch{
    "angular_invalidate_profile", &angular_direct_chain, &angular_feedforward_chain};

inline cogip::motion_control::MetaController<2> linear_pose_loop_meta_controller;
inline cogip::motion_control::MetaController<2> angular_pose_loop_meta_controller;

// PolarParallel for pose loop (linear + angular in parallel)
inline cogip::motion_control::PolarParallelMetaController pose_loop_polar_parallel_meta_controller;

// Throttled pose loop controller (wraps pose_loop_polar_parallel_meta_controller)
// Note: PoseStraightFilter runs at full rate (not throttled) for faster state transitions
inline cogip::motion_control::ThrottledController
    throttled_pose_loop_controllers(&pose_loop_polar_parallel_meta_controller,
                                    feedforward_pose_controllers_throttle_divider);

// ============================================================================
// Speed loop meta controllers (SpeedPID + AntiBlocking)
// ============================================================================

inline cogip::motion_control::MetaController<2> linear_speed_loop_meta_controller;
inline cogip::motion_control::MetaController<2> angular_speed_loop_meta_controller;

// ============================================================================
// SpeedPIDController parameters (use feedforward PIDs)
// ============================================================================

inline cogip::motion_control::SpeedPIDControllerParameters
    linear_speed_controller_parameters(&feedforward_linear_speed_pid);

inline cogip::motion_control::SpeedPIDControllerParameters
    angular_speed_controller_parameters(&feedforward_angular_speed_pid);

// PolarParallel for speed loop (linear + angular in parallel)
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
// QuadPIDMetaController for feedforward chain
// ============================================================================

extern cogip::motion_control::QuadPIDMetaController quadpid_feedforward_meta_controller;

// ============================================================================
// Chain initialization and restore functions
// ============================================================================

/// Initialize feedforward chain meta controller
cogip::motion_control::QuadPIDMetaController* init();

/// Restore feedforward chain to original configuration
void restore();

/// Reset feedforward chain state (acceleration filters and PIDs)
inline void reset()
{
    // Reset acceleration filters
    linear_corrector_accel.reset();
    angular_corrector_accel.reset();

    // Reset PIDs
    feedforward_linear_speed_pid.reset();
    feedforward_angular_speed_pid.reset();
    feedforward_linear_pose_pid.reset();
    feedforward_angular_pose_pid.reset();
}

} // namespace quadpid_feedforward_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
