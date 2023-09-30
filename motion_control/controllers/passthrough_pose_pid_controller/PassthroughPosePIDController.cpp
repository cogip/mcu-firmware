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
    double position_error = this->inputs_[0];

    double position_error_sign = 1;

    if (parameters_->signed_target_speed() && (position_error != 0)) {
        position_error_sign = position_error / fabs(position_error);
    }

    // Compute output values
    double speed_order = parameters_->target_speed() * position_error_sign;

    // Store speed order
    this->outputs_[0] = speed_order;
    // Store current speed (pass through)
    this->outputs_[1] = this->inputs_[1];
    // Store target speed
    this->outputs_[2] = parameters_->target_speed();
};

} // namespace motion_control

} // namespace cogip
