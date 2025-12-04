// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup speed_pid_controller Speed PID controller IO keys default values
/// @{
/// @file
/// @brief Default values for Speed PID controller IO keys.
/// @author Generated from SpeedPIDControllerIOKeys.hpp

#pragma once

#include "SpeedPIDControllerIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Default IO key names for linear SpeedPIDController.
/// Each key is prefixed with "linear_" and set to its corresponding member
/// name.
static const SpeedPIDControllerIOKeys linear_speed_pid_controller_io_keys_default = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .speed_command = "linear_speed_command"};

/// @brief Default IO key names for angular SpeedPIDController.
/// Each key is prefixed with "angular_" and set to its corresponding member
/// name.
static const SpeedPIDControllerIOKeys angular_speed_pid_controller_io_keys_default = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .speed_command = "angular_speed_command"};

} // namespace motion_control

} // namespace cogip

/// @}
