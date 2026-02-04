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
// Chain initialization
// ============================================================================

cogip::motion_control::QuadPIDMetaController* init()
{
    // =========================================================================
    // Linear feedforward chain: PosePID → Combiner → SafetyFilters → SpeedPID
    // =========================================================================
    linear_feedforward_chain.add_controller(&linear_feedforward_pose_controller);
    linear_feedforward_chain.add_controller(&linear_feedforward_combiner_controller);
    linear_feedforward_chain.add_controller(&linear_speed_limit_filter);
    linear_feedforward_chain.add_controller(&linear_acceleration_filter);
    linear_feedforward_chain.add_controller(&linear_feedforward_chain_speed);

    // =========================================================================
    // Angular feedforward chain: PosePID → Combiner → SafetyFilters → SpeedPID
    // =========================================================================
    angular_feedforward_chain.add_controller(&angular_feedforward_pose_controller);
    angular_feedforward_chain.add_controller(&angular_feedforward_combiner_controller);
    angular_feedforward_chain.add_controller(&angular_speed_limit_filter);
    angular_feedforward_chain.add_controller(&angular_acceleration_filter);
    angular_feedforward_chain.add_controller(&angular_feedforward_chain_speed);

    // =========================================================================
    // Linear pose loop: ProfileFeedforward + feedforward chain
    // =========================================================================
    linear_pose_loop_meta_controller.add_controller(&linear_profile_feedforward_controller);
    linear_pose_loop_meta_controller.add_controller(&linear_feedforward_chain);

    // =========================================================================
    // Angular pose loop: ProfileFeedforward + feedforward chain
    // =========================================================================
    angular_pose_loop_meta_controller.add_controller(&angular_profile_feedforward_controller);
    angular_pose_loop_meta_controller.add_controller(&angular_feedforward_chain);

    // =========================================================================
    // Pose loop PolarParallel (linear + angular in parallel)
    // This is throttled - executed at reduced frequency
    // =========================================================================
    pose_loop_polar_parallel_meta_controller.add_controller(&linear_pose_loop_meta_controller);
    pose_loop_polar_parallel_meta_controller.add_controller(&angular_pose_loop_meta_controller);

    // =========================================================================
    // QuadPIDFeedforwardMetaController:
    // PathManagerFilter -> PoseStraightFilter -> Pose loops -> AntiBlocking
    // (Safety filters are now inside feedforward chains, before SpeedPID)
    // =========================================================================
    quadpid_feedforward_meta_controller.add_controller(&path_manager_filter);
    quadpid_feedforward_meta_controller.add_controller(&pose_straight_filter);
    quadpid_feedforward_meta_controller.add_controller(&pose_loop_polar_parallel_meta_controller);

    // Add anti-blocking controllers (common to all configurations)
    // PolarParallel for anti-blocking (linear + angular in parallel)
    speed_loop_polar_parallel_meta_controller.add_controller(&linear_anti_blocking_controller);
    speed_loop_polar_parallel_meta_controller.add_controller(&angular_anti_blocking_controller);
    quadpid_feedforward_meta_controller.add_controller(&speed_loop_polar_parallel_meta_controller);

    // Add telemetry controller for pose data
    quadpid_feedforward_meta_controller.add_controller(&pose_telemetry_controller);

    return &quadpid_feedforward_meta_controller;
}

} // namespace quadpid_feedforward_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
