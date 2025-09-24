// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_pid_controller Speed PID controller IO keys
/// @{
/// @file
/// @brief      Speed PID controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a SpeedPIDController.
struct SpeedPIDControllerIOKeys
{
    etl::string_view speed_error;   ///< e.g. "speed_error"
    etl::string_view current_speed; ///< e.g. "current_speed"
    etl::string_view speed_command; ///< e.g. "speed_command"
};

} // namespace motion_control

} // namespace cogip

/// @}
