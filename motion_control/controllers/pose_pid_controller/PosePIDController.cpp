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

void PosePIDController::execute(ControllersIO& io)
{
    COGIP_DEBUG_COUT("Execute PosePIDController");

    // Read position error (default to 0.0f if missing)
    float position_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_->position_error)) {
        position_error = *opt_err;
    }

    // Compute speed_order via PID
    float speed_order = this->parameters_->pid()->compute(position_error);

    // Write speed order output
    io.set(keys_->speed_order, speed_order);
}

} // namespace motion_control

} // namespace cogip
