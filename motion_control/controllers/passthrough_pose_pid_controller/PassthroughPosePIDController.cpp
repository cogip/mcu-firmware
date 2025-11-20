// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>

// Project includes
#include "etl/absolute.h"
#include "etl/list.h"
#include "etl/vector.h"

#include "log.h"
#include "passthrough_pose_pid_controller/PassthroughPosePIDController.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void PassthroughPosePIDController::execute(ControllersIO& io)
{
    DEBUG("Execute PassthroughPosePIDController\n");

    // Read position error (default to 0.0f if missing)
    float position_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_.position_error)) {
        position_error = *opt_err;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.position_error.data(), position_error);
    }

    // Determine position error sign
    float position_error_sign = 1.0f;
    if (this->parameters_.signed_target_speed() && (position_error != 0.0f)) {
        position_error_sign = position_error / etl::absolute(position_error);
    }

    // Base speed = target_speed from parameters signed by position error
    float speed_order = this->parameters_.target_speed();
    if (position_error != 0.0f) {
        speed_order *= position_error_sign;
    }

    // Write outputs
    io.set(keys_.speed_order, speed_order);
    io.set(keys_.target_speed, this->parameters_.target_speed());
}

} // namespace motion_control

} // namespace cogip
