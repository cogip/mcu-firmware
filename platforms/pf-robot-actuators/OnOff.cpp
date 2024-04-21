// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "OnOff.hpp"
#include "pf_positional_actuators.hpp"

#include <periph/gpio.h>

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

typedef void (*gpio_writer_t)(gpio_t, int value);

void OnOff::disable() {
    actuate(false);
}

void OnOff::actuate(int32_t command) {
    std::cout << "OnOff: Activating LEDs " << command << std::endl;

    command ? gpio_write(pin_, active_state_) : gpio_write(pin_, !active_state_);

    command_ = command;
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
