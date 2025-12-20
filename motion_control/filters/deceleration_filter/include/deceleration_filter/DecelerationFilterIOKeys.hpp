// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    deceleration_filter
/// @{
/// @file
/// @brief      Deceleration filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a DecelerationFilter.
struct DecelerationFilterIOKeys
{
    etl::string_view pose_error;     ///< key for position error (distance to target)
    etl::string_view current_speed;  ///< key for measured current speed
    etl::string_view target_speed;   ///< key for target speed (will be modified)
};

} // namespace motion_control

} // namespace cogip

/// @}
