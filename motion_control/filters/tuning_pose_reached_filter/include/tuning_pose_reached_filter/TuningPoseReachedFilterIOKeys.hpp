// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    tuning_pose_reached_filter
/// @{
/// @file
/// @brief      Tuning pose reached filter IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for a TuningPoseReachedFilter.
struct TuningPoseReachedFilterIOKeys
{
    etl::string_view
        profile_complete; ///< key for profile complete input (from ProfileTrackerController)
    etl::string_view pose_reached; ///< key for pose reached status output
    etl::string_view pose_error;   ///< key for pose error input (optional, for threshold mode)
    float pose_threshold = 0.0f;   ///< threshold for pose_reached (if pose_error is configured)
};

} // namespace motion_control

} // namespace cogip

/// @}
