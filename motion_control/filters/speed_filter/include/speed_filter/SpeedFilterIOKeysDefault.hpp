// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup speed_filter Speed filter IO keys default values
/// @{
/// @file
/// @brief Default values for Speed filter IO keys.
/// @author Generated from SpeedFilterIOKeys.hpp

#pragma once

#include "SpeedFilterIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Default IO key names for linear SpeedFilter.
/// Each key is prefixed with "linear_" and set to its corresponding member
/// name.
static const SpeedFilterIOKeys linear_speed_filter_io_keys_default = {
    .speed_order = "linear_speed_order",
    .current_speed = "linear_current_speed",
    .target_speed = "linear_target_speed",
    .speed_error = "linear_speed_error",
    .bypass_filter = "linear_speed_filter_flag",
};

/// @brief Default IO key names for angular SpeedFilter.
/// Each key is prefixed with "angular_" and set to its corresponding member
/// name.
static const SpeedFilterIOKeys angular_speed_filter_io_keys_default = {
    .speed_order = "angular_speed_order",
    .current_speed = "angular_current_speed",
    .target_speed = "angular_target_speed",
    .speed_error = "angular_speed_error",
    .bypass_filter = "angular_speed_filter_flag",
};

} // namespace motion_control

} // namespace cogip

/// @}
