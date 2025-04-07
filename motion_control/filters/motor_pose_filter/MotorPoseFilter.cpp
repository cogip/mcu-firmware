// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"

#include "motor_pose_filter/MotorPoseFilter.hpp"

namespace cogip {

namespace motion_control {

void MotorPoseFilter::execute() {
    COGIP_DEBUG_COUT("Execute MotorPoseFilter");

    size_t input_index = 0;

    // Current pose
    float current_pose = this->inputs_[input_index++];

    // Target pose
    float target_pose = this->inputs_[input_index++];

    // Current speed
    float current_speed = this->inputs_[input_index++];

    // Target speed
    float target_speed = this->inputs_[input_index++];

    // Position reached flag
    target_pose_status_t pose_reached = (target_pose_status_t)this->inputs_[input_index++];

    if (!is_index_valid(input_index)) {
        std::cerr << __func__ << ": wrong number of inputs" << std::endl;
        return;
    }

    // compute position error
    float pos_err = target_pose - current_pose;

    // Do not disable speed limitation
    bool no_speed_limit = false;

    if (fabs(pos_err) <= parameters_->threshold()) {
        // Reached final pose
        pose_reached = target_pose_status_t::reached;
        target_speed = 0;
    }
    else if (fabs(pos_err) <= ((current_speed * current_speed) / (2 * parameters_->deceleration()))) {
        target_speed = sqrt(2 * parameters_->deceleration() * fabs(pos_err));
    }

    // Pose error
    outputs_[0] = pos_err;
    // Current speed
    outputs_[1] = current_speed;
    // Target speed
    outputs_[2] = fabs(target_speed);
    // Should speed be filtered ?
    outputs_[3] = (float)no_speed_limit;

    // Pose reached
    outputs_[4] = (float)pose_reached;
};

} // namespace motion_control

} // namespace cogip
