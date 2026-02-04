// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    path_manager_filter
/// @{
/// @file
/// @brief      Path manager filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a PathManagerFilter.
struct PathManagerFilterIOKeys
{
    etl::string_view pose_reached;  ///< key for pose reached input (from PoseStraightFilter)
    etl::string_view target_pose_x; ///< key for target X coordinate output
    etl::string_view target_pose_y; ///< key for target Y coordinate output
    etl::string_view target_pose_O; ///< key for target orientation output
    etl::string_view new_target;    ///< key for new target flag output (triggers profile recompute)
    etl::string_view path_complete; ///< key for path complete flag output
    etl::string_view path_index;    ///< key for current path index output (debug)
    etl::string_view bypass_final_orientation; ///< key for bypass final orientation output
    etl::string_view motion_direction;         ///< key for motion direction output
    etl::string_view is_intermediate;          ///< key for intermediate waypoint flag output
};

} // namespace motion_control

} // namespace cogip

/// @}
