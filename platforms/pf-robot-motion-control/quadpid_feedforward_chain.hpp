// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Feedforward chain controller instances
/// @details Controllers that need separate instances for the feedforward chain
///          (cannot be shared with QuadPID chain due to meta-controller ownership).

#pragma once

#include "anti_blocking_controller/AntiBlockingController.hpp"
#include "anti_blocking_controller/AntiBlockingControllerParameters.hpp"
#include "app_conf.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "pose_straight_filter/PoseStraightFilterIOKeysDefault.hpp"
#include "quadpid_chain.hpp"
#include "target_change_detector/TargetChangeDetector.hpp"

namespace cogip {
namespace pf {
namespace motion_control {
namespace feedforward_chain {

// ============================================================================
// PoseStraightFilter (separate instance - cannot be shared between chains)
// Uses same parameters as quadpid_chain to share configuration
// ============================================================================

inline cogip::motion_control::PoseStraightFilter
    pose_straight_filter(cogip::motion_control::pose_straight_filter_io_keys_default,
                         quadpid_chain::pose_straight_filter_parameters);

// ============================================================================
// TargetChangeDetector (separate instance - cannot be shared between chains)
// ============================================================================

inline cogip::motion_control::TargetChangeDetectorIOKeys target_change_detector_io_keys = {
    .target_x = "target_pose_x",
    .target_y = "target_pose_y",
    .target_O = "target_pose_O",
    .new_target = "new_target",
    .current_state = "",
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

} // namespace feedforward_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
