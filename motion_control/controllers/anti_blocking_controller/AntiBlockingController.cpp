// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    anti_blocking_controller
/// @{
/// @file
/// @brief      Anti-Blocking controller implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cstdio>
#include <inttypes.h>

// ETL includes
#include "etl/absolute.h"

// Project includes
#include "anti_blocking_controller/AntiBlockingController.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void AntiBlockingController::execute(ControllersIO& io)
{
    DEBUG("Execute AntiBlockingController\n");

    // Read speed order (default to 0.0 if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        speed_order = *opt;
    } else {
        LOG_WARNING("AntiBlockingController: speed_order not available\n");
    }

    // Read current speed (default to 0.0 if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    } else {
        LOG_WARNING("AntiBlockingController: current_speed not available\n");
    }

    // Compute speed error: speed_order - current_speed
    float speed_error = speed_order - current_speed;

    // Write speed error output (used by SpeedPIDController)
    io.set(keys_.speed_error, speed_error);

    // Anti-blocking detection
    if (parameters_.enabled()) {
        const float speed_threshold = parameters_.speed_threshold();
        const float error_threshold = parameters_.error_threshold();

        // Check if speed is below threshold (robot should be moving but isn't)
        bool below_speed_threshold = etl::absolute(current_speed) < speed_threshold;

        // Check if there's a significant speed error (we're commanding speed but not getting it)
        bool significant_error = etl::absolute(speed_error) > error_threshold;

        if (below_speed_threshold && significant_error) {
            blocked_cycles_count_++;
            DEBUG("Anti-blocking: cycles=%" PRIu16 ", current_speed=%.2f, speed_error=%.2f\n",
                  blocked_cycles_count_, current_speed, speed_error);
        } else {
            blocked_cycles_count_ = 0;
        }

        // Check if we've exceeded the blocking threshold
        if (blocked_cycles_count_ > parameters_.cycles_threshold()) {
            LOG_WARNING("AntiBlockingController: BLOCKED detected\n");
            io.set(keys_.pose_reached, target_pose_status_t::blocked);
        }
    }
}

} // namespace motion_control

} // namespace cogip

/// @}
