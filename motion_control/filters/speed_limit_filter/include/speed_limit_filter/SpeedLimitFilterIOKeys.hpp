// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_limit_filter
/// @{
/// @file
/// @brief      Speed limit filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a SpeedLimitFilter.
struct SpeedLimitFilterIOKeys
{
    etl::string_view target_speed; ///< key for target speed (will be modified)
    etl::string_view
        output_speed; ///< key for output speed (optional, if set writes final value here)
};

} // namespace motion_control

} // namespace cogip

/// @}
