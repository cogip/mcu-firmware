// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Feedforward chain controller definitions and initialization
/// @details Implements the feedforward chain with profile generation,
///          position tracking, and speed control.

#include "quadpid_feedforward_chain.hpp"

#include "app_conf.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerController.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerControllerIOKeys.hpp"
#include "feedforward_combiner_controller/FeedforwardCombinerControllerParameters.hpp"
#include "parameter/Parameter.hpp"
#include "pid/PID.hpp"
#include "pid/PIDParameters.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardControllerIOKeys.hpp"
#include "profile_feedforward_controller/ProfileFeedforwardControllerParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerIOKeysDefault.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeysDefault.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace quadpid_feedforward_chain {

// ============================================================================
// Local PID instances (independent from QUADPID chain)
// ============================================================================

// PID parameters for feedforward chain
static cogip::pid::PIDParameters feedforward_linear_pose_pid_parameters(
    feedforward_linear_pose_pid_kp, feedforward_linear_pose_pid_ki, feedforward_linear_pose_pid_kd,
    feedforward_linear_pose_pid_integral_limit);

static cogip::pid::PIDParameters feedforward_linear_speed_pid_parameters(
    feedforward_linear_speed_pid_kp, feedforward_linear_speed_pid_ki,
    feedforward_linear_speed_pid_kd, feedforward_linear_speed_pid_integral_limit);

static cogip::pid::PIDParameters feedforward_angular_pose_pid_parameters(
    feedforward_angular_pose_pid_kp, feedforward_angular_pose_pid_ki,
    feedforward_angular_pose_pid_kd, feedforward_angular_pose_pid_integral_limit);

static cogip::pid::PIDParameters feedforward_angular_speed_pid_parameters(
    feedforward_angular_speed_pid_kp, feedforward_angular_speed_pid_ki,
    feedforward_angular_speed_pid_kd, feedforward_angular_speed_pid_integral_limit);

// PID instances
cogip::pid::PID feedforward_linear_pose_pid(feedforward_linear_pose_pid_parameters);
cogip::pid::PID feedforward_linear_speed_pid(feedforward_linear_speed_pid_parameters);
cogip::pid::PID feedforward_angular_pose_pid(feedforward_angular_pose_pid_parameters);
cogip::pid::PID feedforward_angular_speed_pid(feedforward_angular_speed_pid_parameters);

// ============================================================================
// ProfileFeedforwardController instances
// ============================================================================

/// Linear ProfileFeedforwardController IO keys
cogip::motion_control::ProfileFeedforwardControllerIOKeys linear_profile_feedforward_io_keys = {
    .pose_error = "linear_pose_error",
    .current_speed = "linear_current_speed",
    .recompute_profile = "linear_recompute_profile",
    .invalidate_profile = "linear_invalidate_profile",
    .feedforward_velocity = "linear_feedforward_velocity",
    .tracking_error = "linear_tracking_error",
    .profile_complete = ""}; // Not used, PoseStraightFilter handles pose_reached

/// Linear ProfileFeedforwardController parameters
cogip::motion_control::ProfileFeedforwardControllerParameters linear_profile_feedforward_parameters(
    platform_max_speed_linear_mm_per_period,      // max_speed
    platform_max_acc_linear_mm_per_period2,       // acceleration
    platform_max_dec_linear_mm_per_period2,       // deceleration
    true,                                         // must_stop_at_end
    feedforward_pose_controllers_throttle_divider // period_increment
);

/// Linear ProfileFeedforwardController
cogip::motion_control::ProfileFeedforwardController
    linear_profile_feedforward_controller(linear_profile_feedforward_io_keys,
                                          linear_profile_feedforward_parameters);

/// Angular ProfileFeedforwardController IO keys
cogip::motion_control::ProfileFeedforwardControllerIOKeys angular_profile_feedforward_io_keys = {
    .pose_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .recompute_profile = "angular_recompute_profile",
    .invalidate_profile = "angular_invalidate_profile",
    .feedforward_velocity = "angular_feedforward_velocity",
    .tracking_error = "angular_tracking_error",
    .profile_complete = ""}; // Not used, PoseStraightFilter handles pose_reached

/// Angular ProfileFeedforwardController parameters
cogip::motion_control::ProfileFeedforwardControllerParameters
    angular_profile_feedforward_parameters(
        platform_max_speed_angular_deg_per_period,    // max_speed
        platform_max_acc_angular_deg_per_period2,     // acceleration
        platform_max_dec_angular_deg_per_period2,     // deceleration (same as acc for angular)
        true,                                         // must_stop_at_end
        feedforward_pose_controllers_throttle_divider // period_increment
    );

/// Angular ProfileFeedforwardController
cogip::motion_control::ProfileFeedforwardController
    angular_profile_feedforward_controller(angular_profile_feedforward_io_keys,
                                           angular_profile_feedforward_parameters);

// ============================================================================
// FeedforwardCombinerController instances
// ============================================================================

/// Linear FeedforwardCombinerController IO keys (position tracker)
/// Outputs speed_order only (not speed_command) - speed combiner will produce speed_command
cogip::motion_control::FeedforwardCombinerControllerIOKeys linear_feedforward_combiner_io_keys = {
    .feedforward_velocity = "linear_feedforward_velocity",
    .feedback_correction = "linear_feedback_correction",
    .speed_order = "linear_speed_order",
    .speed_command = ""}; // Speed combiner will handle this

/// Linear FeedforwardCombinerController parameters
cogip::motion_control::FeedforwardCombinerControllerParameters
    linear_feedforward_combiner_parameters;

/// Linear FeedforwardCombinerController
cogip::motion_control::FeedforwardCombinerController
    linear_feedforward_combiner_controller(linear_feedforward_combiner_io_keys,
                                           linear_feedforward_combiner_parameters);

/// Angular FeedforwardCombinerController IO keys (position tracker)
/// Outputs speed_order only (not speed_command) - speed combiner will produce speed_command
cogip::motion_control::FeedforwardCombinerControllerIOKeys angular_feedforward_combiner_io_keys = {
    .feedforward_velocity = "angular_feedforward_velocity",
    .feedback_correction = "angular_feedback_correction",
    .speed_order = "angular_speed_order",
    .speed_command = ""}; // Speed combiner will handle this

/// Angular FeedforwardCombinerController parameters
cogip::motion_control::FeedforwardCombinerControllerParameters
    angular_feedforward_combiner_parameters;

/// Angular FeedforwardCombinerController
cogip::motion_control::FeedforwardCombinerController
    angular_feedforward_combiner_controller(angular_feedforward_combiner_io_keys,
                                            angular_feedforward_combiner_parameters);

// ============================================================================
// SpeedPIDController instances
// ============================================================================

/// Linear SpeedPIDController
cogip::motion_control::SpeedPIDController linear_feedforward_speed_controller(
    cogip::motion_control::linear_speed_pid_controller_io_keys_default,
    linear_speed_controller_parameters);

/// Angular SpeedPIDController
cogip::motion_control::SpeedPIDController angular_feedforward_speed_controller(
    cogip::motion_control::angular_speed_pid_controller_io_keys_default,
    angular_speed_controller_parameters);

// ============================================================================
// QuadPIDMetaController for feedforward chain
// ============================================================================

cogip::motion_control::QuadPIDMetaController quadpid_feedforward_meta_controller;

// ============================================================================
// Telemetry controller
// ============================================================================

static cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters;

static cogip::motion_control::TelemetryController
    pose_telemetry_controller(cogip::motion_control::linear_telemetry_controller_io_keys_default,
                              telemetry_controller_parameters);

// ============================================================================
// Chain initialization
// ============================================================================

cogip::motion_control::QuadPIDMetaController* init()
{
    // =========================================================================
    // Linear sub-chains for ConditionalSwitch
    // =========================================================================
    // Feedforward chain: PosePID → feedback_correction, Combiner → speed_order
    linear_feedforward_chain.add_controller(&linear_feedforward_pose_controller);
    linear_feedforward_chain.add_controller(&linear_feedforward_combiner_controller);

    // Direct chain: PosePID → AccelerationFilter → DecelerationFilter → speed_order
    linear_direct_chain.add_controller(&linear_position_corrector);
    linear_direct_chain.add_controller(&linear_corrector_accel);
    linear_direct_chain.add_controller(&linear_corrector_speed_limit);
    linear_direct_chain.add_controller(&linear_corrector_decel);

    // =========================================================================
    // Angular sub-chains for ConditionalSwitch
    // =========================================================================
    // Feedforward chain: PosePID → feedback_correction, Combiner → speed_order
    angular_feedforward_chain.add_controller(&angular_feedforward_pose_controller);
    angular_feedforward_chain.add_controller(&angular_feedforward_combiner_controller);

    // Direct chain: PosePID → AccelerationFilter → DecelerationFilter → speed_order
    angular_direct_chain.add_controller(&angular_position_corrector);
    angular_direct_chain.add_controller(&angular_corrector_accel);
    angular_direct_chain.add_controller(&angular_corrector_speed_limit);
    angular_direct_chain.add_controller(&angular_corrector_decel);

    // =========================================================================
    // Linear pose loop (ProfileFeedforward + ConditionalSwitch)
    // ConditionalSwitch handles both PID and Combiner based on invalidate_profile
    // =========================================================================
    linear_pose_loop_meta_controller.add_controller(&linear_profile_feedforward_controller);
    linear_pose_loop_meta_controller.add_controller(&linear_pose_switch);

    // =========================================================================
    // Angular pose loop (ProfileFeedforward + ConditionalSwitch)
    // ConditionalSwitch handles both PID and Combiner based on invalidate_profile
    // =========================================================================
    angular_pose_loop_meta_controller.add_controller(&angular_profile_feedforward_controller);
    angular_pose_loop_meta_controller.add_controller(&angular_pose_switch);

    // =========================================================================
    // Pose loop PolarParallel (linear + angular in parallel)
    // This is throttled - executed at reduced frequency
    // =========================================================================
    pose_loop_polar_parallel_meta_controller.add_controller(&linear_pose_loop_meta_controller);
    pose_loop_polar_parallel_meta_controller.add_controller(&angular_pose_loop_meta_controller);

    // =========================================================================
    // Linear speed loop (SpeedPID + AntiBlocking)
    // SpeedPID does feedforward+feedback internally, outputs speed_command
    // =========================================================================
    linear_speed_loop_meta_controller.add_controller(&linear_feedforward_speed_controller);
    linear_speed_loop_meta_controller.add_controller(&linear_anti_blocking_controller);

    // =========================================================================
    // Angular speed loop (SpeedPID + AntiBlocking)
    // SpeedPID does feedforward+feedback internally, outputs speed_command
    // =========================================================================
    angular_speed_loop_meta_controller.add_controller(&angular_feedforward_speed_controller);
    angular_speed_loop_meta_controller.add_controller(&angular_anti_blocking_controller);

    // =========================================================================
    // Speed loop PolarParallel (linear + angular in parallel)
    // =========================================================================
    speed_loop_polar_parallel_meta_controller.add_controller(&linear_speed_loop_meta_controller);
    speed_loop_polar_parallel_meta_controller.add_controller(&angular_speed_loop_meta_controller);

    // =========================================================================
    // QuadPIDFeedforwardMetaController:
    // PoseStraightFilter (full rate) -> ThrottledController(pose loops) -> Speed loops
    // =========================================================================
    quadpid_feedforward_meta_controller.add_controller(&pose_straight_filter);
    quadpid_feedforward_meta_controller.add_controller(&throttled_pose_loop_controllers);
    quadpid_feedforward_meta_controller.add_controller(&speed_loop_polar_parallel_meta_controller);

    // Add telemetry controller for pose data
    quadpid_feedforward_meta_controller.add_controller(&pose_telemetry_controller);

    return &quadpid_feedforward_meta_controller;
}

// ============================================================================
// Chain restore
// ============================================================================

void restore()
{
    // Linear feedforward profile parameters
    linear_profile_feedforward_parameters.set_max_speed(platform_max_speed_linear_mm_per_period);
    linear_profile_feedforward_parameters.set_acceleration(platform_max_acc_linear_mm_per_period2);
    linear_profile_feedforward_parameters.set_deceleration(platform_max_dec_linear_mm_per_period2);

    // Angular feedforward profile parameters
    angular_profile_feedforward_parameters.set_max_speed(platform_max_speed_angular_deg_per_period);
    angular_profile_feedforward_parameters.set_acceleration(
        platform_max_acc_angular_deg_per_period2);
    angular_profile_feedforward_parameters.set_deceleration(
        platform_max_dec_angular_deg_per_period2);

    // Feedforward speed PID controller parameters
    linear_speed_controller_parameters.set_pid(&feedforward_linear_speed_pid);
    angular_speed_controller_parameters.set_pid(&feedforward_angular_speed_pid);
}

} // namespace quadpid_feedforward_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
