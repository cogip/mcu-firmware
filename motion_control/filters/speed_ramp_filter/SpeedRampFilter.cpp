// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_ramp_filter
/// @{
/// @file
/// @brief      Speed ramp filter implementation
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

// Project includes
#include "speed_ramp_filter/SpeedRampFilter.hpp"
#include "log.h"

#include "etl/absolute.h"

#include <cmath>

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void SpeedRampFilter::execute(ControllersIO& io)
{
    DEBUG("Execute SpeedRampFilter\n");

    // Check if reset is requested via IO
    if (!keys_.reset.empty()) {
        if (auto opt = io.get_as<bool>(keys_.reset)) {
            if (*opt) {
                ramp_setpoint_ = 0.0f;
                io.set(keys_.target_speed, 0.0f);
                DEBUG("[SpeedRampFilter] reset via IO key\n");
                return;
            }
        }
    }

    // Read step setpoint (desired final speed)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    } else {
        LOG_WARNING("SpeedRampFilter: target_speed not available\n");
        return;
    }

    // Read current measured speed (input/feedback)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    } else {
        LOG_WARNING("SpeedRampFilter: current_speed not available\n");
        return;
    }

    float max_acc = parameters_.max_acceleration();
    float max_dec = parameters_.max_deceleration();

    // If input has overshot the ramp (input is on the same side as setpoint
    // relative to ramp), resynchronize the ramp to the current input.
    if ((current_speed - ramp_setpoint_) * (target_speed - ramp_setpoint_) > 0) {
        ramp_setpoint_ = current_speed;
    }

    // Choose acceleration or deceleration rate:
    // - Accelerate if input is moving toward setpoint (same sign as velocity direction)
    // - Decelerate otherwise (input moving away from setpoint)
    float direction = target_speed - current_speed;
    if (current_speed * direction >= 0) {
        ramp_setpoint_ += (direction >= 0 ? 1.0f : -1.0f) * max_acc;
    } else {
        ramp_setpoint_ += (direction >= 0 ? 1.0f : -1.0f) * max_dec;
    }

    // Clamp: ramp must never overshoot the step setpoint
    if ((target_speed - current_speed) * (target_speed - ramp_setpoint_) < 0) {
        ramp_setpoint_ = target_speed;
    }

    // Distance-based braking: v_brake = sqrt(2 * deceleration * distance)
    if (!keys_.pose_error.empty()) {
        if (auto opt = io.get_as<float>(keys_.pose_error)) {
            float abs_pose_error = etl::absolute(*opt);
            float braking_speed = std::sqrt(2.0f * max_dec * abs_pose_error);

            if (etl::absolute(ramp_setpoint_) > braking_speed) {
                float sign = (ramp_setpoint_ >= 0.0f) ? 1.0f : -1.0f;
                ramp_setpoint_ = sign * braking_speed;
            }
        }
    }

    DEBUG("SpeedRampFilter: setpoint=%.2f input=%.2f ramp=%.2f\n",
          static_cast<double>(target_speed), static_cast<double>(current_speed),
          static_cast<double>(ramp_setpoint_));

    // Write filtered ramp output back as target speed
    io.set(keys_.target_speed, ramp_setpoint_);
}

} // namespace motion_control

} // namespace cogip

/// @}
