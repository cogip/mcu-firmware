// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_ramp_filter
/// @{
/// @file
/// @brief      Speed ramp filter IO keys
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a SpeedRampFilter.
struct SpeedRampFilterIOKeys
{
    etl::string_view target_speed;  ///< key for step setpoint (desired final speed)
    etl::string_view current_speed; ///< key for current measured speed (input/feedback)
    etl::string_view pose_error;    ///< key for remaining distance to goal (for braking)
    etl::string_view reset;         ///< key for reset signal (clears ramp state)
};

} // namespace motion_control

} // namespace cogip

/// @}
