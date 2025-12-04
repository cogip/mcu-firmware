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

    // Check state gating (if configured)
    bool state_gating_enabled = !keys_.current_state.empty();
    if (state_gating_enabled) {
        int current_state = -1;
        if (auto opt = io.get_as<int>(keys_.current_state)) {
            current_state = *opt;
        } else {
            LOG_WARNING("FeedforwardCombinerController: current_state not available\n");
        }

        // If not in active state, output zero and return
        if (current_state != keys_.active_state) {
            DEBUG("Not in active state (current=%d, active=%d), outputting zero\n", current_state,
                  keys_.active_state);
            io.set(keys_.speed_order, 0.0f);
            return;
        }
    }

    // Read feedforward velocity (default to 0.0 if missing)
    float feedforward_velocity = 0.0f;
    if (auto opt = io.get_as<float>(keys_.feedforward_velocity)) {
        feedforward_velocity = *opt;
    } else {
        LOG_WARNING(
            "FeedforwardCombinerController: feedforward_velocity not available, using 0.0\n");
    }

    // Read feedback correction (default to 0.0 if missing)
    float feedback_correction = 0.0f;
    if (auto opt = io.get_as<float>(keys_.feedback_correction)) {
        feedback_correction = *opt;
    } else {
        LOG_WARNING(
            "FeedforwardCombinerController: feedback_correction not available, using 0.0\n");
    }

    // Combine: speed_order = feedforward + feedback
    float speed_order = feedforward_velocity + feedback_correction;

    DEBUG("Combiner: feedforward_velocity=%.2f + feedback_correction=%.2f = speed_order=%.2f\n",
          feedforward_velocity, feedback_correction, speed_order);

    // Write speed order output
    io.set(keys_.speed_order, speed_order);
}

} // namespace motion_control

} // namespace cogip

/// @}
