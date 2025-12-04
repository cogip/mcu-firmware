// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file
/// @brief Feedforward chain controller instances
/// @details Controllers that need separate instances for the feedforward chain
///          (cannot be shared with QuadPID chain due to meta-controller ownership).

#pragma once

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

} // namespace feedforward_chain
} // namespace motion_control
} // namespace pf
} // namespace cogip
