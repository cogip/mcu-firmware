// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>

// Project includes
#include "etl/list.h"
#include "etl/vector.h"

#include "log.h"
#include "pose_pid_controller/PosePIDController.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void PosePIDController::execute(ControllersIO& io)
{
    DEBUG("Execute PosePIDController\n");

    // Read position error (default to 0.0f if missing)
    float position_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_.position_error)) {
        position_error = *opt_err;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.position_error.data(), position_error);
    }

    // Compute speed_order via PID
    float speed_order = this->parameters_.pid()->compute(position_error);

    // Write speed order output
    io.set(keys_.speed_order, speed_order);
}

} // namespace motion_control

} // namespace cogip
