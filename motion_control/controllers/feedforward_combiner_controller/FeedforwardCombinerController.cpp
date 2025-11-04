// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    feedforward_combiner_controller
/// @{
/// @file
/// @brief      Feedforward Combiner controller implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cstdio>

// Project includes
#include "feedforward_combiner_controller/FeedforwardCombinerController.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void FeedforwardCombinerController::execute(ControllersIO& io)
{
    DEBUG("Execute FeedforwardCombinerController");

    // Read feedforward velocity (default to 0.0 if missing)
    double feedforward_velocity = 0.0;
    if (auto opt = io.get_as<double>(keys_.feedforward_velocity)) {
        feedforward_velocity = *opt;
    }
    else {
        LOG_WARNING("FeedforwardCombinerController: feedforward_velocity not available, using 0.0\n");
    }

    // Read feedback correction (default to 0.0 if missing)
    double feedback_correction = 0.0;
    if (auto opt = io.get_as<double>(keys_.feedback_correction)) {
        feedback_correction = *opt;
    }
    else {
        LOG_WARNING("FeedforwardCombinerController: feedback_correction not available, using 0.0\n");
    }

    // Combine: speed_order = feedforward + feedback
    double speed_order = feedforward_velocity + feedback_correction;

    DEBUG("Combiner: ff=%.2f + fb=%.2f = %.2f\n", feedforward_velocity, feedback_correction, speed_order);

    // Write speed order output
    io.set(keys_.speed_order, speed_order);
}

} // namespace motion_control

} // namespace cogip

/// @}
