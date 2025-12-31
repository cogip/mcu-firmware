// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    acceleration_filter
/// @{
/// @file
/// @brief      Acceleration filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for an AccelerationFilter.
struct AccelerationFilterIOKeys
{
    etl::string_view target_speed; ///< key for target speed input
    etl::string_view
        output_speed; ///< key for output speed (optional, if empty modifies target_speed in-place)
};

} // namespace motion_control

} // namespace cogip

/// @}
