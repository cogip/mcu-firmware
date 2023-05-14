// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
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
    if (motor_driver_ >= MOTOR_DRIVER_NUMOF) {
        std::cerr << "Motor " << id_ << ": wrong motor driver" << std::endl;
        return;
    }

    const motor_driver_config_t *config = &motor_driver_config[motor_driver_];

    if (motor_id_ >= config->nb_motors) {
        std::cerr << "Motor " << id_ << ": wrong motor id" << std::endl;
        return;
    }

    motor_set(motor_driver_, motor_id_, 0);
    motor_disable(motor_driver_, motor_id_);
}

bool Motor::disable_on_check() {
    int direction = (command_ < 0) ? -1 : 1;

    // Check limit switches
    if ((direction > 0) && check_limit_switch_positive_direction_cb_ && check_limit_switch_positive_direction_cb_()) {
        motor_disable(motor_driver_, motor_id_);
        std::cerr << "Motor " << id_ << ": positive limit" << std::endl;
        return true;
    }
    if ((direction < 0) && check_limit_switch_negative_direction_cb_ && check_limit_switch_negative_direction_cb_()) {
        motor_disable(motor_driver_, motor_id_);
        std::cerr << "Motor " << id_ << ": negative limit" << std::endl;
        return true;
    }

    return false;
}

void Motor::actuate(int32_t command) {
    command_ = command;
    int speed = abs(command);

    if (speed > 100) {
        std::cerr << "Motor " << id_ << ": speed is not in range [-100%:100%]" << std::endl;
        return;
    }

    if (motor_driver_ >= MOTOR_DRIVER_NUMOF) {
        std::cerr << "Motor " << id_ << ": wrong motor driver" << std::endl;
        return;
    }

    const motor_driver_config_t *config = &motor_driver_config[motor_driver_];

    if (motor_id_ >= config->nb_motors) {
        std::cerr << "Motor " << id_ << ": wrong motor id" << std::endl;
        return;
    }

    if (disable_on_check()) {
        return;
    }

    int32_t pwm_duty_cycle =
        (int32_t)(config->pwm_resolution * command) / 100;

    motor_set(motor_driver_, motor_id_, pwm_duty_cycle);
    if (pwm_duty_cycle) {
        motor_enable(motor_driver_, motor_id_);
    }
    else {
        motor_disable(motor_driver_, motor_id_);
    }
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
