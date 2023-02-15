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

    // Store speed order
    this->outputs_[0] = this->inputs_[2];
    // Store current speed (pass through)
    this->outputs_[1] = this->inputs_[1];
    // Store target speed (pass through)
    this->outputs_[2] = this->inputs_[2];
};

} // namespace motion_control

} // namespace cogip
