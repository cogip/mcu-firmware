// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    anti_blocking_controller Anti-Blocking controller IO keys
/// @{
/// @file
/// @brief      Anti-Blocking controller IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for AntiBlockingController.
///
/// This controller:
/// 1. Computes speed_error = speed_order - current_speed
/// 2. Detects blocking conditions (low speed despite command)
/// 3. Sets pose_reached to blocked if blocking detected
struct AntiBlockingControllerIOKeys
{
    etl::string_view speed_order;   ///< Input: commanded speed (e.g. "linear_speed_order")
    etl::string_view current_speed; ///< Input: measured speed (e.g. "linear_current_speed")
    etl::string_view
        speed_error; ///< Output: speed_order - current_speed (e.g. "linear_speed_error")
    etl::string_view pose_reached; ///< Output: pose reached status (written to blocked if detected)
};

} // namespace motion_control

} // namespace cogip

/// @}
