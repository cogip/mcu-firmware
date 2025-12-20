// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_pid_controller Pose PID controller IO keys
/// @{
/// @file
/// @brief      Pose PID controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a PosePIDController.
///
/// Optional state gating: if current_state is set, the controller only executes
/// when current_state matches active_state. Otherwise it returns early
/// without writing (preserving any previous value in speed_order).
struct PosePIDControllerIOKeys
{
    etl::string_view position_error; ///< e.g. "pose_error"
    etl::string_view current_speed;  ///< e.g. "current_speed"
    etl::string_view target_speed;   ///< e.g. "target_speed"
    etl::string_view disable_filter; ///< e.g. "disable_speed_filter"
    etl::string_view pose_reached;   ///< e.g. "pose_reached"
    etl::string_view speed_order;    ///< e.g. "speed_order"
    etl::string_view current_state;  ///< e.g. "pose_straight_filter_state" (optional gating)
    int active_state = -1; ///< State value when controller should be active (-1 = always active)
};

} // namespace motion_control

} // namespace cogip

/// @}
