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

void PassthroughPosePIDController::execute(ControllersIO& io) {
    COGIP_DEBUG_COUT("Execute PassthroughPosePIDController");

    // Read position error
    float position_error = *io.get_as<float>("position_error");

    float position_error_sign = 1;

    if (parameters_->signed_target_speed() && (position_error != 0)) {
        position_error_sign = position_error / fabs(position_error);
    }

    float speed_order = parameters_->target_speed();
    if (position_error != 0) {
        // Compute output values
        speed_order *= position_error_sign;
    }

    io.set("speed_order",  speed_order);
    io.set("target_speed", parameters_->target_speed());
};

} // namespace motion_control

} // namespace cogip
