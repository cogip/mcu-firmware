#include "speed_pid_controller/SpeedPIDController.hpp"
#include <iostream>

namespace cogip {

namespace motion_control {

void SpeedPIDController::execute(ControllersIO& io)
{
    std::cout << "Start SpeedPIDController" << std::endl;

    // Read speed error (default to 0.0f if missing)
    float speed_error = 0.0f;
    if (auto opt_err = io.get_as<float>(keys_.speed_error)) {
        speed_error = *opt_err;
    }
    else {
        std::cout   << "WARNING: " << keys_.speed_error
                    << " is not available, using default value "
                    << speed_error << std::endl;
    }

    // Compute speed command via PID
    float speed_command = this->parameters_.pid()->compute(speed_error);

    // Write speed command
    io.set(keys_.speed_command, speed_command);

    std::cout << "End SpeedPIDController" << std::endl;
}

}  // namespace motion_control

}  // namespace cogip
