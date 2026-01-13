// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_limit_filter
/// @{
/// @file
/// @brief      Speed limit filter implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cmath>

// ETL includes
#include "etl/absolute.h"

// Project includes
#include "log.h"
#include "speed_limit_filter/SpeedLimitFilter.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void SpeedLimitFilter::execute(ControllersIO& io)
{
    DEBUG("Execute SpeedLimitFilter\n");

    // Read target speed (default to 0.0 if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    } else {
        LOG_WARNING("SpeedLimitFilter: target_speed not available\n");
        return;
    }

    float min_speed = parameters_.min_speed();
    float max_speed = parameters_.max_speed();
    float abs_target_speed = etl::absolute(target_speed);
    float sign = (target_speed >= 0.0f) ? 1.0f : -1.0f;

    DEBUG("SpeedLimitFilter: target_speed=%.2f, min=%.2f, max=%.2f\n",
          static_cast<double>(target_speed), static_cast<double>(min_speed),
          static_cast<double>(max_speed));

    // Apply maximum speed limit
    if (abs_target_speed > max_speed) {
        DEBUG("SpeedLimitFilter: limiting to max_speed %.2f\n", static_cast<double>(max_speed));
        target_speed = sign * max_speed;
    }

    // Apply minimum speed limit (only if speed is non-zero)
    // This prevents the robot from moving too slowly (motor deadband)
    if (abs_target_speed > 0.0f && abs_target_speed < min_speed) {
        DEBUG("SpeedLimitFilter: boosting to min_speed %.2f\n", static_cast<double>(min_speed));
        target_speed = sign * min_speed;
    }

    // Write modified target speed back to IO
    io.set(keys_.target_speed, target_speed);

    // If output_speed key is defined, write final target_speed there
    if (!keys_.output_speed.empty()) {
        io.set(keys_.output_speed, target_speed);
    }
}

} // namespace motion_control

} // namespace cogip

/// @}
