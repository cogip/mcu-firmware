// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Pose test chain implementation

#include "pose_test_chain.hpp"
#include "linear_pose_tuning_chain.hpp"
#include "motion_control.hpp"
#include "motion_control_common/MetaController.hpp"
#include "parameter/Parameter.hpp"
#include "pid/PID.hpp"
#include "pid/PIDParameters.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerIOKeys.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "quadpid_feedforward_chain.hpp"
#include "telemetry_controller/TelemetryController.hpp"
#include "telemetry_controller/TelemetryControllerIOKeysDefault.hpp"
#include "telemetry_controller/TelemetryControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace pose_test_chain {

// ============================================================================
// Meta controllers and telemetry
// ============================================================================

static cogip::motion_control::TelemetryControllerParameters telemetry_controller_parameters;

static cogip::motion_control::TelemetryController
    linear_telemetry_controller(cogip::motion_control::linear_telemetry_controller_io_keys_default,
                                telemetry_controller_parameters);

static cogip::motion_control::TelemetryController angular_telemetry_controller(
    cogip::motion_control::angular_telemetry_controller_io_keys_default,
    telemetry_controller_parameters);

// ============================================================================
// P-only PID for angular correction (no I, no D to avoid oscillations)
// ============================================================================

static Parameter<float, ReadOnly> null_ki{0};
static Parameter<float, ReadOnly> null_kd{0};
static Parameter<float, ReadOnly> null_integral_limit{0};

static cogip::pid::PIDParameters angular_p_only_parameters(angular_pose_pid_kp, null_ki, null_kd,
                                                           null_integral_limit);

static cogip::pid::PID angular_p_only_pid(angular_p_only_parameters);

// ============================================================================
// Pose controllers
// ============================================================================

static cogip::motion_control::PosePIDControllerParameters
    linear_pose_controller_parameters(&pose_test_linear_pose_pid);

static cogip::motion_control::PosePIDController
    linear_pose_controller(linear_pose_tuning_chain::pose_pid_io_keys,
                           linear_pose_controller_parameters);

// Angular pose controller IO keys (simplified: direct pose_error -> speed_setpoint)
static cogip::motion_control::PosePIDControllerIOKeys angular_pose_io_keys = {
    .position_error = "angular_pose_error",
    .current_speed = "angular_current_speed",
    .target_speed = "",
    .disable_filter = "",
    .pose_reached = "",
    .speed_order = "angular_speed_setpoint"};

static cogip::motion_control::PosePIDControllerParameters
    angular_pose_controller_parameters(&angular_p_only_pid);

static cogip::motion_control::PosePIDController
    angular_pose_controller(angular_pose_io_keys, angular_pose_controller_parameters);

// ============================================================================
// Separate instances of ProfileFeedforward and Combiner for pose_test_chain
// (cannot share with quadpid_feedforward_chain - controller ownership)
// ============================================================================

// ProfileFeedforward for pose test (separate instance)
static cogip::motion_control::ProfileFeedforwardControllerIOKeys
    pose_test_linear_profile_feedforward_io_keys{.pose_error = "linear_pose_error",
                                                 .current_speed = "linear_current_speed",
                                                 .recompute_profile = "linear_recompute_profile",
                                                 .feedforward_velocity =
                                                     "linear_feedforward_velocity",
                                                 .tracking_error = "linear_tracking_error",
                                                 .profile_complete = "linear_profile_complete"};

static cogip::motion_control::ProfileFeedforwardControllerParameters
    pose_test_linear_profile_feedforward_parameters(
        platform_max_speed_linear_mm_per_period, // max_speed
        platform_max_acc_linear_mm_per_period2,  // acceleration
        platform_max_dec_linear_mm_per_period2,  // deceleration
        true,                                    // must_stop_at_end
        1                                        // period_increment
    );

static cogip::motion_control::ProfileFeedforwardController
    pose_test_linear_profile_feedforward_controller(
        pose_test_linear_profile_feedforward_io_keys,
        pose_test_linear_profile_feedforward_parameters);

// FeedforwardCombiner for pose test (separate instance)
static cogip::motion_control::FeedforwardCombinerControllerIOKeys
    pose_test_linear_feedforward_combiner_io_keys{
        .feedforward_velocity = "linear_feedforward_velocity",
        .feedback_correction = "linear_feedback_correction",
        .speed_order = "linear_speed_order",
        .speed_command = "linear_speed_command"};

static cogip::motion_control::FeedforwardCombinerControllerParameters
    pose_test_linear_feedforward_combiner_parameters;

static cogip::motion_control::FeedforwardCombinerController
    pose_test_linear_feedforward_combiner_controller(
        pose_test_linear_feedforward_combiner_io_keys,
        pose_test_linear_feedforward_combiner_parameters);

// ============================================================================
// Initialization function
// ============================================================================

cogip::motion_control::MetaController<>* init()
{
    // Linear pose loop: PoseErrorFilter -> ProfileFeedforward -> PosePID -> Combiner
    linear_pose_loop_meta_controller.add_controller(&linear_pose_error_filter);
    linear_pose_loop_meta_controller.add_controller(
        &pose_test_linear_profile_feedforward_controller);
    linear_pose_loop_meta_controller.add_controller(&linear_pose_controller);
    linear_pose_loop_meta_controller.add_controller(
        &pose_test_linear_feedforward_combiner_controller);

    // Angular pose loop (simplified): PoseErrorFilter -> PosePID (P-only)
    angular_pose_loop_meta_controller.add_controller(&angular_pose_error_filter);
    angular_pose_loop_meta_controller.add_controller(&angular_pose_controller);

    // Pose loop polar parallel (linear + angular in parallel)
    pose_loop_polar_parallel_meta_controller.add_controller(&linear_pose_loop_meta_controller);
    pose_loop_polar_parallel_meta_controller.add_controller(&angular_pose_loop_meta_controller);

    // Linear speed controller
    linear_speed_meta_controller.add_controller(&linear_speed_controller);

    // Angular speed controller
    angular_speed_meta_controller.add_controller(&angular_speed_controller);

    // Speed loop polar parallel (linear + angular in parallel)
    speed_loop_polar_parallel_meta_controller.add_controller(&linear_speed_meta_controller);
    speed_loop_polar_parallel_meta_controller.add_controller(&angular_speed_meta_controller);

    // Main meta controller chain
    meta_controller.add_controller(&pose_loop_polar_parallel_meta_controller);
    meta_controller.add_controller(&speed_loop_polar_parallel_meta_controller);
    meta_controller.add_controller(&tuning_pose_reached_filter);
    meta_controller.add_controller(&linear_telemetry_controller);
    meta_controller.add_controller(&angular_telemetry_controller);

    return &meta_controller;
}

} // namespace pose_test_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
