// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    feedforward_combiner_controller Feedforward Combiner controller IO keys
/// @{
/// @file
/// @brief      Feedforward Combiner controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for FeedforwardCombinerController.
///
/// This controller simply adds feedforward velocity and feedback correction.
struct FeedforwardCombinerControllerIOKeys
{
    etl::string_view feedforward_velocity; ///< e.g. "linear_feedforward_velocity"
    etl::string_view feedback_correction;  ///< e.g. "linear_feedback_correction"
    etl::string_view speed_order;          ///< e.g. "linear_speed_order" (output for telemetry)
    etl::string_view speed_command;        ///< e.g. "linear_speed_command" (output for motors)
};

} // namespace motion_control

} // namespace cogip

/// @}
