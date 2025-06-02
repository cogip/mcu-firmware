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

void PassthroughPosePIDController::execute(ControllersIO& io)
{
    COGIP_DEBUG_COUT("Execute PassthroughPosePIDController");

    // Read position error (default to 0.0f if missing)
    float position_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_->position_error)) {
        position_error = *opt_err;
    }

    // Determine position error sign
    float position_error_sign = 1.0f;
    if (this->parameters_->signed_target_speed() && (position_error != 0.0f)) {
        position_error_sign = position_error / std::fabs(position_error);
    }

    // Base speed = target_speed from parameters signed by position error
    float speed_order = this->parameters_->target_speed();
    if (position_error != 0.0f) {
        speed_order *= position_error_sign;
    }

    // Write outputs
    io.set(keys_->speed_order,  speed_order);
    io.set(keys_->target_speed, this->parameters_->target_speed());
}

} // namespace motion_control

} // namespace cogip
