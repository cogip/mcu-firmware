// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    deceleration_filter
/// @{
/// @file
/// @brief      Deceleration filter implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cmath>

// ETL includes
#include "etl/absolute.h"

// Project includes
#include "deceleration_filter/DecelerationFilter.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void DecelerationFilter::execute(ControllersIO& io)
{
    DEBUG("Execute DecelerationFilter\n");

    // Read pose error (default to 0.0 if missing)
    float pose_error = 0.0f;
    if (auto opt = io.get_as<float>(keys_.pose_error)) {
        pose_error = *opt;
    } else {
        LOG_WARNING("DecelerationFilter: pose_error not available\n");
        return;
    }

    // Read current speed (default to 0.0 if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    } else {
        LOG_WARNING("DecelerationFilter: current_speed not available\n");
    }

    // Read speed order (default to 0.0 if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        speed_order = *opt;
    } else {
        LOG_WARNING("DecelerationFilter: speed_order '%.*s' not available\n",
                    static_cast<int>(keys_.target_speed.size()), keys_.target_speed.data());
    }

    float deceleration = parameters_.deceleration();
    float abs_pose_error = etl::absolute(pose_error);
    float abs_current_speed = etl::absolute(current_speed);
    float abs_speed_order = etl::absolute(speed_order);

    // (previously used for verbose logging)

    // Compute braking distance: d = v² / (2 * a)
    float braking_distance = (abs_current_speed * abs_current_speed) / (2.0f * deceleration);

    DEBUG("DecelerationFilter: pose_error=%.2f, current_speed=%.2f, speed_order=%.2f, "
          "braking_dist=%.2f\n",
          static_cast<double>(pose_error), static_cast<double>(current_speed),
          static_cast<double>(speed_order), static_cast<double>(braking_distance));

    // If remaining distance is less than braking distance, we need to decelerate
    if (abs_pose_error <= braking_distance) {
        // Compute deceleration speed: v = sqrt(2 * a * d)
        float decel_speed = std::sqrt(2.0f * deceleration * abs_pose_error);

        // Limit speed order to deceleration speed (only if it's lower)
        if (decel_speed < abs_speed_order) {
            DEBUG("DecelerationFilter: limiting speed_order from %.2f to %.2f\n",
                  static_cast<double>(abs_speed_order), static_cast<double>(decel_speed));
            // Preserve sign of speed_order
            float sign = (speed_order >= 0.0f) ? 1.0f : -1.0f;
            speed_order = sign * decel_speed;
        }
    }

    // Verbose log removed to reduce log spam in production.

    // Write filtered speed_order back
    io.set(keys_.target_speed, speed_order);
}

} // namespace motion_control

} // namespace cogip

/// @}
