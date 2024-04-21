// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"
#include "thread/thread.hpp"

#include "platform_engine/PlatformEngine.hpp"

namespace cogip {

namespace motion_control {

void PlatformEngine::prepare_inputs() {
    // Update current pose and speed
    platform_get_speed_and_pose_cb_(current_speed_, current_pose_);

    if (controller_) {
        size_t index = 0;

        // Current pose
        controller_->set_input(index++, current_pose_.x());
        controller_->set_input(index++, current_pose_.y());
        controller_->set_input(index++, current_pose_.O());

        // Target pose
        controller_->set_input(index++, target_pose_.x());
        controller_->set_input(index++, target_pose_.y());
        controller_->set_input(index++, target_pose_.O());

        // Current speed
        controller_->set_input(index++, current_speed_.distance());
        controller_->set_input(index++, current_speed_.angle());

        // Target speed
        controller_->set_input(index++, target_speed_.distance());
        controller_->set_input(index++, target_speed_.angle());

        // Allow reverse
        controller_->set_input(index++, target_pose_.allow_reverse());

        if (index != controller_->nb_inputs()) {
            std::cerr << "PlatformEngine: Wrong number of inputs, " << index << " given, " << controller_->nb_inputs() << " expected." << std::endl;
        }
    }
};

void PlatformEngine::process_outputs() {
    // If timeout is enabled, pose_reached_ has been set by the engine itself, do not override it.
    if (!timeout_enable_)
        pose_reached_ = (target_pose_status_t)controller_->output(2);

    cogip_defs::Polar command(
        controller_->output(0),
        controller_->output(1)
    );

    platform_process_commands_cb_(command);
};

} // namespace motion_control

} // namespace cogip
