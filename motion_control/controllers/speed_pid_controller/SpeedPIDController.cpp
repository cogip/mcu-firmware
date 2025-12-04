#include "speed_pid_controller/SpeedPIDController.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void SpeedPIDController::execute(ControllersIO& io)
{
    DEBUG("Start SpeedPIDController\n");

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

    DEBUG("End SpeedPIDController\n");
}

} // namespace motion_control

} // namespace cogip
