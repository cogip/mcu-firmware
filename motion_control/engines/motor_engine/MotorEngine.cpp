// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"
#include "thread/thread.hpp"

#include "motor_engine/MotorEngine.hpp"

namespace cogip {

namespace motion_control {

void MotorEngine::prepare_inputs() {
    // Update current distance and speed
    odometer_.update();
    float current_distance = odometer_.distance_mm();
    float current_speed = odometer_.delta_distance_mm();

    if (controller_) {
        size_t index = 0;

        // Current distance
        controller_->set_input(index++, current_distance);

        // Target distance
        controller_->set_input(index++, target_distance_);

        // Current speed
        controller_->set_input(index++, current_speed);

        // Target speed
        controller_->set_input(index++, target_speed_);

        // Position reached flag
        controller_->set_input(index++, target_pose_status_t::moving);

        if (index != controller_->nb_inputs()) {
            std::cerr << "MotorEngine: Wrong number of inputs, " << index << " given, " << controller_->nb_inputs() << " expected.";
        }
    }
};

void MotorEngine::process_outputs() {
    // If timeout is enabled, pose_reached_ has been set by the engine itself, do not override it.
    if (pose_reached_ != target_pose_status_t::timeout) {
        pose_reached_ = (target_pose_status_t)controller_->output(1);
    }
    else {
        std::cerr << "MotorEngine timed out, disable." << std::endl;

        // Disable engine
        enable_ = false;

        // Disable motor
        motor_.disable();

        return;
    }

    if (pose_reached_ == target_pose_status_t::blocked) {
        std::cerr << "MotorEngine blocked, disable." << std::endl;

        // Disable engine
        enable_ = false;

        // Disable motor
        motor_.disable();

        return;
    }

    // Disable the timeout as we want to hold the position
    if ((pose_reached_ == target_pose_status_t::reached)
        && (timeout_enable_ == true)) {
        // Reset timeout cycles counter
        timeout_cycle_counter_ = timeout_ms_ / engine_thread_period_ms_;
    }

    int command = (int)controller_->output(0);

    motor_.set_speed(command);
};

} // namespace motion_control

} // namespace cogip
