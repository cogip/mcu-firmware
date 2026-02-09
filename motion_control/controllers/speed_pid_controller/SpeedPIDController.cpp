#include "speed_pid_controller/SpeedPIDController.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void SpeedPIDController::execute(ControllersIO& io)
{
    DEBUG("Start SpeedPIDController\n");

    // Check if reset is requested
    if (auto opt = io.get_as<bool>(keys_.reset)) {
        if (*opt) {
            // Reset the PID (clears integral term)
            this->parameters_.pid()->reset();
            // Clear the reset flag
            io.set(keys_.reset, false);
            DEBUG("[SpeedPID %s] reset via IO key\n", keys_.reset.data());
        }
    }

    // Read speed order (default to 0.0f if missing)
    float speed_order = 0.0f;
    if (auto opt = io.get_as<float>(keys_.speed_order)) {
        speed_order = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.speed_order.data(), speed_order);
    }

    // Read current speed (default to 0.0f if missing)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.current_speed.data(), current_speed);
    }

    // Compute speed error
    float speed_error = speed_order - current_speed;

    // Compute speed command via PID
    float speed_command = this->parameters_.pid()->compute(speed_error);

    // Write speed command
    io.set(keys_.speed_command, speed_command);

    DEBUG("SpeedPID: order=%.2f cur=%.2f err=%.2f cmd=%.2f\n", static_cast<double>(speed_order),
          static_cast<double>(current_speed), static_cast<double>(speed_error),
          static_cast<double>(speed_command));
}

} // namespace motion_control

} // namespace cogip
