// System includes
#include <cmath>
#include <iostream>

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
    raw_target = std::min(raw_target, max_speed);

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
    COGIP_DEBUG_COUT("Execute SpeedFilter");

    // Read commanded speed before filtering (default to zero if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_->speed_order)) {
        speed_order = *opt;
    }

    // Read measured current speed (default to zero if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_->current_speed)) {
        current_speed = *opt;
    }

    // Read raw target speed (default to zero if missing)
    float target_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_->target_speed)) {
        target_speed = *opt;
    }

    // Read flag disabling filtering (default to false if missing)
    bool no_filter = false;
    if (auto opt = io.get_as<float>(keys_->no_speed_filter)) {
        no_filter = static_cast<bool>(*opt);
    }

    // Read incoming pose‐reached status (default to moving if missing)
    target_pose_status_t pose_reached = target_pose_status_t::moving;
    if (auto opt = io.get_as<float>(keys_->pose_reached_in)) {
        pose_reached = static_cast<target_pose_status_t>(static_cast<int>(*opt));
    }

    if (!no_filter) {
        limit_speed_order(
            &speed_order,
            target_speed,
            parameters_->min_speed(),
            parameters_->max_speed(),
            parameters_->max_acceleration()
        );
    }

    if (parameters_->anti_blocking()) {
        bool below_threshold = std::fabs(current_speed) < parameters_->anti_blocking_speed_threshold();
        bool no_acceleration = std::fabs(previous_speed_order_ - current_speed) > parameters_->anti_blocking_error_threshold();

        if (below_threshold && no_acceleration) {
            anti_blocking_blocked_cycles_nb_++;
            std::cout << "Anti blocking cycles number: " << anti_blocking_blocked_cycles_nb_ << std::endl;
        }
        else {
            anti_blocking_blocked_cycles_nb_ = 0;
        }

        if (anti_blocking_blocked_cycles_nb_ > parameters_->anti_blocking_blocked_cycles_nb_threshold()) {
            std::cout << "BLOCKED" << std::endl;
            pose_reached = target_pose_status_t::blocked;
        }
    }

    previous_speed_order_ = speed_order;

    float speed_error = speed_order - current_speed;

    // Write computed speed error
    io.set(keys_->speed_error, speed_error);

    // Write updated pose‐reached status
    io.set(keys_->pose_reached_out, static_cast<float>(pose_reached));
}

} // namespace motion_control

} // namespace cogip
