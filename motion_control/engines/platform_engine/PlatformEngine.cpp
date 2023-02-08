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
    platform_get_speed_and_pose_cb_(current_speed_, current_pose_);

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

    if (index != controller_->nb_inputs()) {
        COGIP_DEBUG_CERR("PlatformEngine: Wrong number of inputs, " << index << " given, " << controller_->nb_inputs() << " expected.");
    }
};

void PlatformEngine::process_outputs() {
};

} // namespace motion_control

} // namespace cogip
