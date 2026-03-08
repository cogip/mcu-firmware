// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    tracker_combiner_controller Tracker Combiner controller IO keys
/// @{
/// @file
/// @brief      Tracker Combiner controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for TrackerCombinerController.
///
/// This controller simply adds tracker velocity and feedback correction.
struct TrackerCombinerControllerIOKeys
{
    etl::string_view tracker_velocity;    ///< e.g. "linear_tracker_velocity"
    etl::string_view feedback_correction; ///< e.g. "linear_feedback_correction"
    etl::string_view speed_order;         ///< e.g. "linear_speed_order" (output for telemetry)
    etl::string_view speed_command;       ///< e.g. "linear_speed_command" (output for motors)
};

} // namespace motion_control

} // namespace cogip

/// @}
