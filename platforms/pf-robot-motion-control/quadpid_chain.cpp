// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief QuadPID chain controller initialization
/// @details Implements the classic cascaded PID chain initialization.

#include "quadpid_chain.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace quadpid_chain {

// ============================================================================
// Chain initialization
// ============================================================================

cogip::motion_control::QuadPIDMetaController* init()
{
    // Linear pose loop meta controller (pose controller only, executed at reduced frequency)
    linear_pose_loop_meta_controller.add_controller(&linear_pose_controller);

    // Linear speed loop meta controller (speed filter + anti-blocking + speed controller)
    linear_speed_loop_meta_controller.add_controller(&linear_speed_filter);
    linear_speed_loop_meta_controller.add_controller(&linear_anti_blocking_controller);
    linear_speed_loop_meta_controller.add_controller(&linear_speed_controller);

    // Angular pose loop meta controller (pose controller only, executed at reduced frequency)
    angular_pose_loop_meta_controller.add_controller(&angular_pose_controller);

    // Angular speed loop meta controller (speed filter + anti-blocking + speed controller)
    angular_speed_loop_meta_controller.add_controller(&angular_speed_filter);
    angular_speed_loop_meta_controller.add_controller(&angular_anti_blocking_controller);
    angular_speed_loop_meta_controller.add_controller(&angular_speed_controller);

    // Pose loop PolarParallelMetaController (pose controllers only)
    // --> Linear pose loop meta controller
    // `-> Angular pose loop meta controller
    pose_loop_polar_parallel_meta_controller.add_controller(&linear_pose_loop_meta_controller);
    pose_loop_polar_parallel_meta_controller.add_controller(&angular_pose_loop_meta_controller);

    // Pose loop meta controller (pose_straight_filter + deceleration filters + pose loop polar
    // parallel) PoseStraightFilter -> DecelerationFilters -> Pose loop PolarParallelMetaController
    pose_loop_meta_controller.add_controller(&pose_straight_filter);
    pose_loop_meta_controller.add_controller(&linear_deceleration_filter);
    pose_loop_meta_controller.add_controller(&angular_deceleration_filter);
    pose_loop_meta_controller.add_controller(&pose_loop_polar_parallel_meta_controller);

    // Speed loop PolarParallelMetaController (speed controllers only)
    // --> Linear speed loop meta controller
    // `-> Angular speed loop meta controller
    speed_loop_polar_parallel_meta_controller.add_controller(&linear_speed_loop_meta_controller);
    speed_loop_polar_parallel_meta_controller.add_controller(&angular_speed_loop_meta_controller);

    // QuadPIDMetaController:
    // PathManagerFilter -> ThrottledController(pose_loop_meta_controller, 10) -> Speed loop
    quadpid_meta_controller.add_controller(&path_manager_filter);
    quadpid_meta_controller.add_controller(&throttled_pose_loop_controllers);
    quadpid_meta_controller.add_controller(&speed_loop_polar_parallel_meta_controller);

    return &quadpid_meta_controller;
}

} // namespace quadpid_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
