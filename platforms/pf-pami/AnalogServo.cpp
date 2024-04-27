// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "AnalogServo.hpp"
#include "pf_positional_actuators.hpp"

#include <periph/gpio.h>
#include <pcf857x.h>

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

pca9685_t AnalogServo::pca9685_dev;

AnalogServo::AnalogServo(
    Enum id,
    uint32_t default_timeout_period,
    int channel
) : PositionalActuator(id, default_timeout_period), channel_(channel) {
}

void AnalogServo::add_position(uint16_t position) {
    if (!positions_.full()) {
        positions_.push_back(position);
    }
    else {
        std::cerr << "ERROR: Servomotor " << channel_ << " positions list is full!" << std::endl;
    }
}

void AnalogServo::disable() {
    pca9685_pwm_set(&AnalogServo::pca9685_dev, channel_, 0);
}

void AnalogServo::actuate(int32_t command) {
    if ((command >= 0) && (static_cast<unsigned int>(command) < positions_.size())) {
        pca9685_pwm_set(&AnalogServo::pca9685_dev, channel_, positions_.at(command));
        std::cout << "INFO: Servomotor " << channel_ << " at position " << positions_.at(command) << std::endl;
    }
    else {
        std::cerr << "ERROR: Servomotor " << channel_ << " command out of range!" << std::endl;
    }

    command_ = command;
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
