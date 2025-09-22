// System includes
#include <cmath>
#include <iostream>
#include "etl/algorithm.h"

// Project includes
#include "speed_filter/SpeedFilter.hpp"

namespace cogip {

namespace motion_control {

void SpeedFilter::limit_speed_order(
    float *speed_order,
    float raw_target,
    float min_speed,
    float max_speed,
    float max_acc
)
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
        }
        else if (acceleration < 0.0f) {
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
    std::cout << "Execute SpeedFilter" << std::endl;

    // Read commanded speed before filtering (default to zero if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        speed_order = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.speed_order
                    << " is not available, using default value "
                    << speed_order << std::endl;
    }

    // Read measured current speed (default to zero if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.current_speed
                    << " is not available, using default value "
                    << current_speed << std::endl;
    }

    // Read raw target speed (default to zero if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_speed)) {
        target_speed = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.target_speed
                    << " is not available, using default value "
                    << target_speed << std::endl;
    }

    // Read flag disabling filtering (default to false if missing)
    bool no_filter = false;
    if (auto opt = io.get_as<bool>(keys_.speed_filter_flag)) {
        no_filter = *opt;
    }
    else {
        std::cout   << "WARNING: " << keys_.speed_filter_flag
                    << " is not available, using default value "
                    << no_filter << std::endl;
    }

    if (!no_filter) {
        limit_speed_order(
            &speed_order,
            target_speed,
            parameters_.min_speed(),
            parameters_.max_speed(),
            parameters_.max_acceleration()
        );
    }

    if (parameters_.anti_blocking()) {
        const float anti_blocking_speed_threshold = parameters_.anti_blocking_speed_threshold();
        const float anti_blocking_error_threshold = parameters_.anti_blocking_error_threshold();
        bool below_threshold = etl::absolute(current_speed) < anti_blocking_speed_threshold;
        bool no_acceleration = etl::absolute(previous_speed_order_ - current_speed) > anti_blocking_error_threshold;

        if (below_threshold && no_acceleration) {
            anti_blocking_blocked_cycles_nb_++;
            std::cout << "Anti blocking cycles number: " << anti_blocking_blocked_cycles_nb_ << std::endl;
        }
        else {
            anti_blocking_blocked_cycles_nb_ = 0;
        }

        if (anti_blocking_blocked_cycles_nb_ > parameters_.anti_blocking_blocked_cycles_nb_threshold()) {
            std::cout << "BLOCKED" << std::endl;

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
