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

    // Check state gating (if configured)
    bool state_gating_enabled = !keys_.current_state.empty();
    if (state_gating_enabled) {
        int current_state = -1;
        if (auto opt = io.get_as<int>(keys_.current_state)) {
            current_state = *opt;
        } else {
            LOG_WARNING("PosePIDController: current_state not available\n");
        }

        // If not in active state, output zero and return
        if (current_state != keys_.active_state) {
            DEBUG("PosePIDController: Not in active state (current=%d, active=%d), outputting zero "
                  "(key=%s)\n",
                  current_state, keys_.active_state, keys_.speed_order.data());
            io.set(keys_.speed_order, 0.0f);
            return;
        }
    }

    // Read position error (default to 0.0f if missing)
    float position_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_.position_error)) {
        position_error = *opt_err;
    } else {
        LOG_WARNING("PosePIDController: %s is not available, using default value %f\n",
                    keys_.position_error.data(), position_error);
    }

    // Compute speed_order via PID
    float speed_order = this->parameters_.pid()->compute(position_error);

    // Write speed order output
    io.set(keys_.speed_order, speed_order);

    DEBUG("PosePIDController: position_error=%.2f -> speed_order=%.2f (key=%s)\n",
          static_cast<double>(position_error), static_cast<double>(speed_order),
          keys_.speed_order.data());
}

} // namespace motion_control

} // namespace cogip
