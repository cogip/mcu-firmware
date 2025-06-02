#include "speed_pid_controller/SpeedPIDController.hpp"
#include <iostream>

namespace cogip {

namespace motion_control {

void SpeedPIDController::execute(ControllersIO& io)
{
    COGIP_DEBUG_COUT("Execute SpeedPIDController");

    // Read speed error (default to 0.0f if missing)
    float speed_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_->speed_error)) {
        speed_error = *opt_err;
    }

    // Compute speed command via PID
    float speed_command = this->parameters_->pid()->compute(speed_error);

    // Write speed command
    io.set(keys_->speed_command, speed_command);

    // “current_speed” remains as-is in ControllersIO (pass-through)
}

}  // namespace motion_control

}  // namespace cogip
