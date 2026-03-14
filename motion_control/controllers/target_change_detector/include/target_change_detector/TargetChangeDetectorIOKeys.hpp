// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    target_change_detector Target Change Detector IO keys
/// @{
/// @file
/// @brief      Target Change Detector IO keys
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/array.h>
#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief Bundle of ControllersIO key names for TargetChangeDetector.
///
/// This controller detects when any of the watched IO keys changes and sets
/// a flag to trigger profile regeneration in downstream controllers.
/// Set any watched key to empty string to skip it.
///
/// @tparam MAX_KEYS Maximum number of IO keys to watch (default 4)
template <size_t MAX_KEYS = 4> struct TargetChangeDetectorIOKeys
{
    etl::array<etl::string_view, MAX_KEYS> watched_keys = {}; ///< IO keys to watch (empty to skip)
    etl::string_view new_target;                              ///< e.g. "new_target" (output flag)
};

} // namespace motion_control

} // namespace cogip

/// @}
