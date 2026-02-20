// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_error_filter
/// @{
/// @file
/// @brief      Pose error filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a PoseErrorFilter.
struct PoseErrorFilterIOKeys
{
    etl::string_view target_x;  ///< key for target X coordinate (linear mode)
    etl::string_view target_y;  ///< key for target Y coordinate (linear mode)
    etl::string_view target_O;  ///< key for target orientation (angular mode)
    etl::string_view current_x; ///< key for current X coordinate (linear mode)
    etl::string_view current_y; ///< key for current Y coordinate (linear mode)
    etl::string_view current_O; ///< key for current orientation (linear mode for direction, angular
                                ///< mode for error)
    etl::string_view pose_error; ///< key for computed pose error output
    etl::string_view new_target; ///< key for new target flag (read from TargetChangeDetector)
};

} // namespace motion_control

} // namespace cogip

/// @}
