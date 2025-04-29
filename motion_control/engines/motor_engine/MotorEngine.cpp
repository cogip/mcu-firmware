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
    // Update current pose and speed
    motor_get_speed_and_pose_cb_(current_speed_, current_pose_);

    if (controller_) {
        size_t index = 0;

        // Current pose
        controller_->set_input(index++, current_pose_);

        // Target pose
        controller_->set_input(index++, target_pose_);

        // Current speed
        controller_->set_input(index++, current_speed_);

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
        std::cerr << "MotorEngine timed out, brake." << std::endl;

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
