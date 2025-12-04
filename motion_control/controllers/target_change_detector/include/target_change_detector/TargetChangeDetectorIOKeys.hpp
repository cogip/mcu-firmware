// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    target_change_detector Target Change Detector IO keys
/// @{
/// @file
/// @brief      Target Change Detector IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for TargetChangeDetector.
///
/// This controller detects when the target pose changes and sets a flag
/// to trigger profile regeneration in downstream controllers.
/// Uses target_x, target_y, and target_O to detect pose changes.
/// Set any key to empty string to skip that coordinate.
///
/// Optionally, can also trigger on state transitions:
/// - Set current_state to read the state machine state
/// - Set trigger_state to the state value that should trigger new_target
struct TargetChangeDetectorIOKeys
{
    etl::string_view target_x;   ///< e.g. "target_pose_x" (X coordinate, empty to skip)
    etl::string_view target_y;   ///< e.g. "target_pose_y" (Y coordinate, empty to skip)
    etl::string_view target_O;   ///< e.g. "target_pose_O" (orientation, empty to skip)
    etl::string_view new_target; ///< e.g. "new_target" (output flag)
    etl::string_view
        current_state; ///< e.g. "pose_straight_filter_state" (optional, empty to disable)
    int trigger_state; ///< State value that triggers new_target (e.g. MOVE_TO_POSITION=1)
};

} // namespace motion_control

} // namespace cogip

/// @}
