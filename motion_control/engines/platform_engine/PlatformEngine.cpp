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
    size_t index = 0;

    // Update current pose and speed
    platform_get_poses_cb_(this->current_pose_, this->target_pose_);
    platform_get_speeds_cb_(this->current_speed_, this->target_speed_);

    // Current pose
    controller_->set_input(index++, this->current_pose_.x());
    controller_->set_input(index++, this->current_pose_.y());
    controller_->set_input(index++, this->current_pose_.O());

    // Target pose
    controller_->set_input(index++, this->target_pose_.x());
    controller_->set_input(index++, this->target_pose_.y());
    controller_->set_input(index++, this->target_pose_.O());

    // Current speed
    controller_->set_input(index++, this->current_speed_.distance());
    controller_->set_input(index++, this->current_speed_.angle());

    // Target speed
    controller_->set_input(index++, this->target_speed_.distance());
    controller_->set_input(index++, this->target_speed_.angle());

    if (index != controller_->nb_inputs()) {
        std::cerr << "Wrong number of inputs, " << index << " given, " << controller_->nb_inputs() << " expected." << std::endl;
    }
};

void PlatformEngine::process_outputs() {
};

} // namespace motion_control

} // namespace cogip
