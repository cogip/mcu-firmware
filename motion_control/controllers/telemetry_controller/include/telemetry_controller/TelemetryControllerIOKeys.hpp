// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    telemetry_controller Telemetry controller IO keys
/// @{
/// @file
/// @brief      Telemetry controller IO keys
/// @author     Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a TelemetryController.
struct TelemetryControllerIOKeys
{
    etl::string_view speed_order;      ///< e.g. "speed_order"
    etl::string_view current_speed;    ///< e.g. "current_speed"
    etl::string_view speed_command;    ///< e.g. "speed_command"
    etl::string_view tracker_velocity; ///< e.g. "tracker_velocity"
};

} // namespace motion_control

} // namespace cogip

/// @}
