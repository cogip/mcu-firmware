// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Adaptive Pure Pursuit chain controller initialization
/// @details Implements the Adaptive Pure Pursuit chain initialization.

#include "adaptive_pure_pursuit_chain.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace adaptive_pure_pursuit_chain {

// ============================================================================
// Chain initialization
// ============================================================================

cogip::motion_control::QuadPIDMetaController* init()
{
    // Angular pose loop meta controller for ROTATING_TO_FINAL state
    // (ProfileTracker → PosePID → Combiner)
    angular_pose_loop_meta_controller.add_controller(&angular_profile_tracker_controller);
    angular_pose_loop_meta_controller.add_controller(&angular_pose_pid_controller);
    angular_pose_loop_meta_controller.add_controller(&angular_tracker_combiner_controller);

    // noop_controller does nothing (for FOLLOWING_PATH state)

    // Linear speed loop meta controller (anti-blocking + speed controller)
    linear_speed_loop_meta_controller.add_controller(&linear_anti_blocking_controller);
    linear_speed_loop_meta_controller.add_controller(&linear_speed_controller);

    // Angular speed loop meta controller (anti-blocking + speed controller)
    angular_speed_loop_meta_controller.add_controller(&angular_anti_blocking_controller);
    angular_speed_loop_meta_controller.add_controller(&angular_speed_controller);

    // Speed loop PolarParallelMetaController (linear + angular in parallel)
    speed_loop_polar_parallel_meta_controller.add_controller(&linear_speed_loop_meta_controller);
    speed_loop_polar_parallel_meta_controller.add_controller(&angular_speed_loop_meta_controller);

    // Main QuadPIDMetaController:
    // AdaptivePurePursuitController → ConditionalSwitch(pose_loop) → Speed loop → Telemetry
    meta_controller.add_controller(&pure_pursuit_controller);
    meta_controller.add_controller(&rotating_in_place_conditional_switch);
    meta_controller.add_controller(&speed_loop_polar_parallel_meta_controller);
    meta_controller.add_controller(&telemetry_controller);

    return &meta_controller;
}

} // namespace adaptive_pure_pursuit_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
