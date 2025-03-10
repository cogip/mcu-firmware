// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "cogip_defs/Polar.hpp"
#include "etl/list.h"
#include "etl/vector.h"

#include "speed_pid_controller/SpeedPIDController.hpp"

namespace cogip {

namespace motion_control {

void SpeedPIDController::execute() {
    COGIP_DEBUG_COUT("Execute SpeedPIDController");

    // Speed error
    float speed_error = this->inputs_[0];

    // Compute output values.
    float speed_command = parameters_->pid()->compute(speed_error);
    
    ztimer_now_t current_time = ztimer_now(ZTIMER_MSEC);

    std::cout << controller_name_ << "SpeedCommand:" << std::dec << current_time << ":" << speed_command << std::endl;

    // Store output values.
    this->outputs_[0] = speed_command;
    // Pose reached
    this->outputs_[1] = this->inputs_[1];
};

} // namespace motion_control

} // namespace cogip
