// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_filter Speed filter IO keys
/// @{
/// @file
/// @brief      Speed filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a SpeedFilter.
///        The application must supply the correct literals at runtime.
struct SpeedFilterIOKeys {
    etl::string_view speed_order;              ///< key for commanded speed before filtering
    etl::string_view current_speed;            ///< key for measured current speed
    etl::string_view target_speed;             ///< key for raw speed setpoint
    etl::string_view speed_filter_flag;        ///< key for flag disabling filtering
    etl::string_view speed_error;              ///< key for computed speed error (filtered)

    etl::string_view pose_reached;             ///< key for incoming pose‐reached status
};
} // namespace motion_control

} // namespace cogip

/// @}
