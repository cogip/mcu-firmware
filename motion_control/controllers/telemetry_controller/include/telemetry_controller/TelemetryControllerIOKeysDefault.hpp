// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup telemetry_controller Telemetry controller IO keys default values
/// @{
/// @file
/// @brief Default values for Telemetry controller IO keys.
/// @author Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

#include "TelemetryControllerIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Default IO key names for linear TelemetryController.
/// Each key is prefixed with "linear_" and set to its corresponding member
/// name.
static const TelemetryControllerIOKeys linear_telemetry_controller_io_keys_default = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .speed_command = "linear_speed_command",
    .tracker_velocity = "linear_tracker_velocity"};

/// @brief Default IO key names for angular TelemetryController.
/// Each key is prefixed with "angular_" and set to its corresponding member
/// name.
static const TelemetryControllerIOKeys angular_telemetry_controller_io_keys_default = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .speed_command = "angular_speed_command",
    .tracker_velocity = "angular_tracker_velocity"};

} // namespace motion_control

} // namespace cogip

/// @}
