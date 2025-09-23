// System includes
#include "etl/absolute.h"
#include "etl/algorithm.h"
#include <cmath>

// RIOT includes
#include "log.h"
#include <inttypes.h>

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
    DEBUG("Execute SpeedFilter");

    // Read commanded speed before filtering (default to zero if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        speed_order = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.speed_order.data(), speed_order);
    }

    // Read measured current speed (default to zero if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_speed.data(), current_speed);
    }

    // Read raw target speed (default to zero if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_speed.data(), target_speed);
    }

    // Read flag disabling filtering (default to false if missing)
    bool no_filter = false;
    if (auto opt = io.get_as<bool>(keys_.speed_filter_flag)) {
        no_filter = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %d",
                    keys_.speed_filter_flag.data(), no_filter);
    }

    if (!no_filter) {
        limit_speed_order(&speed_order, target_speed, parameters_.min_speed(),
                          parameters_.max_speed(), parameters_.max_acceleration());
    }

    if (parameters_.anti_blocking()) {
        const float anti_blocking_speed_threshold = parameters_.anti_blocking_speed_threshold();
        const float anti_blocking_error_threshold = parameters_.anti_blocking_error_threshold();
        bool below_threshold = etl::absolute(current_speed) < anti_blocking_speed_threshold;
        bool no_acceleration =
            etl::absolute(previous_speed_order_ - current_speed) > anti_blocking_error_threshold;

        if (below_threshold && no_acceleration) {
            anti_blocking_blocked_cycles_nb_++;
            DEBUG("Anti blocking cycles number: %" PRIu32,
                  static_cast<uint32_t>(anti_blocking_blocked_cycles_nb_));
        } else {
            anti_blocking_blocked_cycles_nb_ = 0;
        }

        if (anti_blocking_blocked_cycles_nb_ >
            parameters_.anti_blocking_blocked_cycles_nb_threshold()) {
            LOG_WARNING("BLOCKED");

            // Write updated pose‐reached status
            io.set(keys_.pose_reached, target_pose_status_t::blocked);
        }
    }

    previous_speed_order_ = speed_order;

    float speed_error = speed_order - current_speed;

    // Write computed speed error
    io.set(keys_.speed_error, speed_error);
}

} // namespace motion_control

} // namespace cogip
