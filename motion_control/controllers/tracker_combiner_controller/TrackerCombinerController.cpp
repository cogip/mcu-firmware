// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    tracker_combiner_controller
/// @{
/// @file
/// @brief      Tracker Combiner controller implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cstdio>

// Project includes
#include "log.h"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void TrackerCombinerController::execute(ControllersIO& io)
{
    DEBUG("Execute TrackerCombinerController");

    // Read tracker velocity (default to 0.0 if missing)
    float tracker_velocity = 0.0f;
    if (auto opt = io.get_as<float>(keys_.tracker_velocity)) {
        tracker_velocity = *opt;
    } else {
        LOG_WARNING("TrackerCombinerController: tracker_velocity not available, using 0.0\n");
    }

    // Read feedback correction (default to 0.0 if missing)
    float feedback_correction = 0.0f;
    if (auto opt = io.get_as<float>(keys_.feedback_correction)) {
        feedback_correction = *opt;
    } else {
        LOG_WARNING("TrackerCombinerController: feedback_correction not available, using 0.0\n");
    }

    // Combine: speed_order = tracker + feedback
    float speed_order = tracker_velocity + feedback_correction;

    DEBUG("Combiner: tracker_velocity=%.2f + feedback_correction=%.2f = speed_order=%.2f\n",
          tracker_velocity, feedback_correction, speed_order);

    // Write speed order output (tracker + feedback correction)
    // This goes to SpeedPID which will produce speed_command
    if (!keys_.speed_order.empty()) {
        io.set(keys_.speed_order, speed_order);
    }

    // Write speed command output (same value, alternative key name)
    if (!keys_.speed_command.empty()) {
        io.set(keys_.speed_command, speed_order);
    }
}

} // namespace motion_control

} // namespace cogip

/// @}
