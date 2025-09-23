#include "speed_pid_controller/SpeedPIDController.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void SpeedPIDController::execute(ControllersIO& io)
{
    DEBUG("Start SpeedPIDController\n");

    // Read speed error (default to 0.0f if missing)
    float speed_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_.speed_error)) {
        speed_error = *opt_err;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.speed_error.data(), speed_error);
    }

    // Compute speed command via PID
    float speed_command = this->parameters_.pid()->compute(speed_error);

    // Write speed command
    io.set(keys_.speed_command, speed_command);

    DEBUG("End SpeedPIDController\n");
}

} // namespace motion_control

} // namespace cogip
