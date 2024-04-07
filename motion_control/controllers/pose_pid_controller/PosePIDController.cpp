// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"

#include "pose_pid_controller/PosePIDController.hpp"

namespace cogip {

namespace motion_control {

void PosePIDController::execute() {
    COGIP_DEBUG_COUT("Execute PosePIDController");

    // Read position error.
    double position_error = this->inputs_[0];

    // Compute output values.
    double speed_order = parameters_->pid()->compute(position_error);

    // Store speed order
    this->outputs_[0] = speed_order;
    // Store current speed (pass through)
    this->outputs_[1] = this->inputs_[1];
    // Store target speed (pass through)
    this->outputs_[2] = this->inputs_[2];
    // Store disabling speed filter (pass through)
    this->outputs_[3] = this->inputs_[3];
};

} // namespace motion_control

} // namespace cogip
