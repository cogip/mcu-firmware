// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    tuning_pose_reached_filter
/// @{
/// @file
/// @brief      Tuning pose reached filter implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// Project includes
#include "tuning_pose_reached_filter/TuningPoseReachedFilter.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void TuningPoseReachedFilter::execute(ControllersIO& io)
{
    DEBUG("Execute TuningPoseReachedFilter\n");

    // Check if profile is complete
    if (auto opt = io.get_as<bool>(keys_.profile_complete)) {
        if (*opt) {
            io.set(keys_.pose_reached, target_pose_status_t::reached);
            DEBUG("TuningPoseReachedFilter: pose reached (profile complete)\n");
        }
    }
}

} // namespace motion_control

} // namespace cogip

/// @}
