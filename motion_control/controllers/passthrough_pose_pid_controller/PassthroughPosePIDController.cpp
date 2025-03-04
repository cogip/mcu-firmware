// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"

#include "passthrough_pose_pid_controller/PassthroughPosePIDController.hpp"

namespace cogip {

namespace motion_control {

void PassthroughPosePIDController::execute() {
    COGIP_DEBUG_COUT("Execute PassthroughPosePIDController");

    // Read position error
    float position_error = this->inputs_[0];

    float position_error_sign = 1;

    if (parameters_->signed_target_speed() && (position_error != 0)) {
        position_error_sign = position_error / fabs(position_error);
    }

    float speed_order = parameters_->target_speed();
    if (position_error != 0) {
        // Compute output values
        speed_order *= position_error_sign;
    }

    // Store speed order
    this->outputs_[0] = speed_order;
    // Store current speed (pass through)
    this->outputs_[1] = this->inputs_[1];
    // Store target speed
    this->outputs_[2] = parameters_->target_speed();
    // Store disabling speed filter (pass through)
    this->outputs_[3] = this->inputs_[3];
    // Pose reached (pass through)
    this->outputs_[4] = this->inputs_[4];
};

} // namespace motion_control

} // namespace cogip
