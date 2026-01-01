// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    acceleration_filter
/// @{
/// @file
/// @brief      Acceleration filter implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cmath>

// ETL includes
#include "etl/absolute.h"
#include "etl/algorithm.h"

// Project includes
#include "acceleration_filter/AccelerationFilter.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void AccelerationFilter::execute(ControllersIO& io)
{
    DEBUG("Execute AccelerationFilter\n");

    // Read speed order (default to 0.0 if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        speed_order = *opt;
    } else {
        LOG_WARNING("AccelerationFilter: speed_order not available\n");
        return;
    }

    float acceleration = parameters_.acceleration();
    float min_speed = parameters_.min_speed();

    float abs_previous_speed = etl::absolute(previous_speed_order_);
    float abs_speed_order = etl::absolute(speed_order);

    // (previously used for verbose logging)

    // If speed order is greater than previous output speed, limit the increase
    if (abs_speed_order > abs_previous_speed) {
        // Maximum allowed speed = previous_speed + acceleration (per period)
        // But never less than min_speed (guaranteed startup speed)
        float max_allowed_speed = etl::max(abs_previous_speed + acceleration, min_speed);

        // Limit speed order to max allowed speed
        if (abs_speed_order > max_allowed_speed) {
            // Preserve sign of speed_order
            float sign = (speed_order >= 0.0f) ? 1.0f : -1.0f;
            speed_order = sign * max_allowed_speed;
        }
    }

    // Apply min_speed threshold: if output is between -min_speed and +min_speed,
    // snap to ±min_speed based on acceleration direction (like SpeedFilter)
    float abs_output = etl::absolute(speed_order);
    if (abs_output > 0.0f && abs_output < min_speed) {
        float acceleration_direction = speed_order - previous_speed_order_;
        if (acceleration_direction > 0.0f) {
            speed_order = min_speed;
        } else if (acceleration_direction < 0.0f) {
            speed_order = -min_speed;
        }
    }

    // Verbose logs removed to reduce log spam in production.

    // Save output speed for next cycle
    previous_speed_order_ = speed_order;

    // Write filtered speed_order back
    io.set(keys_.target_speed, speed_order);
}

} // namespace motion_control

} // namespace cogip

/// @}
