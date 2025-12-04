// System includes
#include "etl/algorithm.h"
#include <cmath>

// RIOT includes
#include "log.h"

// Project includes
#include "speed_filter/SpeedFilter.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void SpeedFilter::limit_speed_order(float* speed_order, float raw_target, float min_speed,
                                    float max_speed, float max_acc)
{
    raw_target = etl::min(raw_target, max_speed);

    float acceleration = *speed_order - previous_speed_order_;
    if (acceleration > max_acc) {
        acceleration = max_acc;
    }
    if (acceleration < -max_acc) {
        acceleration = -max_acc;
    }

    *speed_order = previous_speed_order_ + acceleration;

    if (*speed_order < min_speed && *speed_order > -min_speed) {
        if (acceleration > 0.0f) {
            *speed_order = min_speed;
        } else if (acceleration < 0.0f) {
            *speed_order = -min_speed;
        }
    }

    if (*speed_order > raw_target) {
        *speed_order = raw_target;
    }
    if (*speed_order < -raw_target) {
        *speed_order = -raw_target;
    }
}

void SpeedFilter::execute(ControllersIO& io)
{
    // Read commanded speed before filtering (default to zero if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        speed_order = *opt;
    }

    // Read measured current speed (default to zero if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    }

    // Read raw target speed (default to zero if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    }

    DEBUG("SpeedFilter[%s]: speed_order=%.2f, current_speed=%.2f, target_speed=%.2f, prev=%.2f\n",
          keys_.speed_order.data(), static_cast<double>(speed_order),
          static_cast<double>(current_speed), static_cast<double>(target_speed),
          static_cast<double>(previous_speed_order_));

    // Apply speed/acceleration limits
    limit_speed_order(&speed_order, target_speed, parameters_.min_speed(), parameters_.max_speed(),
                      parameters_.max_acceleration());

    previous_speed_order_ = speed_order;

    float speed_error = speed_order - current_speed;

    // Write computed speed error
    io.set(keys_.speed_error, speed_error);
}

} // namespace motion_control

} // namespace cogip
