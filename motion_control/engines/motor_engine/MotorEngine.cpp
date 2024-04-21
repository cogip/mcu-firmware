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
    if (!timeout_enable_)
        pose_reached_ = (target_pose_status_t)controller_->output(1);

    int command = (int)controller_->output(0);

    motor_process_commands_cb_(command, *this);
};

} // namespace motion_control

} // namespace cogip
