// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "board.h"
#include "Motor.hpp"

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

void Motor::disable() {
    if (!motor_driver_) {
        std::cerr << __func__ << ": motor " << id_ << ": motor driver is null pointer" << std::endl;
        return;
    }

    if (motor_id_ >= motor_driver_->params->nb_motors) {
        std::cerr << __func__ << ": motor " << id_ << ": wrong motor id" << std::endl;
        return;
    }

    motor_set(motor_driver_, motor_id_, 0);
    motor_disable(motor_driver_, motor_id_);
}

bool Motor::disable_on_check() {
    return true;
}

void Motor::actuate(int32_t command) {
    command_ = command;
    int speed = abs(command);

    if (speed > 100) {
        std::cerr << __func__ << ": motor " << id_ << ": speed is not in range [-100%:100%]" << std::endl;
        return;
    }

    if (!motor_driver_) {
        std::cerr << __func__ << ": motor " << id_ << ": motor driver is null pointer" << std::endl;
        return;
    }

    if (motor_id_ >= motor_driver_->params->nb_motors) {
        std::cerr << __func__ << ": motor " << id_ << ": wrong motor id" << std::endl;
        return;
    }

    if (disable_on_check()) {
        return;
    }

    int32_t pwm_duty_cycle =
        (int32_t)(motor_driver_->params->pwm_resolution * command) / 100;

    motor_set(motor_driver_, motor_id_, pwm_duty_cycle);
    if (pwm_duty_cycle) {
        motor_enable(motor_driver_, motor_id_);
    }
    else {
        motor_disable(motor_driver_, motor_id_);
    }

    if (!timeout_period_)
        timeout_period_ = default_timeout_period_;
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
