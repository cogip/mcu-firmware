// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "LxMotor.hpp"
#include "lx_servo.h"

#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

uart_half_duplex_t * LxMotor::lx_stream = nullptr;

LxMotor::LxMotor(
    Enum id,
    GroupEnum group,
    uint8_t order,
    uint32_t default_timeout_period,
    lx_id_t lx_id,
    check_limit_switch_cb_t check_limit_switch_positive_direction_cb,
    check_limit_switch_cb_t check_limit_switch_negative_direction_cb
) : PositionalActuator(id, group, order, default_timeout_period),
    lx_id_(lx_id),
    check_limit_switch_positive_direction_cb_(check_limit_switch_positive_direction_cb),
    check_limit_switch_negative_direction_cb_(check_limit_switch_negative_direction_cb) {
    lx_init(&lx_, lx_stream, lx_id_);
}

void LxMotor::disable() {
    // Just set speed to 0
    lx_servo_or_motor_mode_write(&lx_, false, 0);
}

bool LxMotor::disable_on_check() {
    int direction = (command_ < 0) ? -1 : 1;

    // Check limit switches
    if ((direction > 0) && check_limit_switch_positive_direction_cb_ && check_limit_switch_positive_direction_cb_()) {
        disable();
        std::cerr << "LxMotor " << id_ << ": positive limit" << std::endl;
        return true;
    }
    if ((direction < 0) && check_limit_switch_negative_direction_cb_ && check_limit_switch_negative_direction_cb_()) {
        disable();
        std::cerr << "LxMotor " << id_ << ": negative limit" << std::endl;
        return true;
    }

    return false;
}

void LxMotor::actuate(int32_t command) {
    command_ = command;
    int speed = abs(command);

    if (speed > 100) {
        std::cerr << "LxMotor " << id_ << ": speed is not in range [-100%:100%]" << std::endl;
        return;
    }

    if (disable_on_check()) {
        return;
    }

    // lx_servo speed range is [-1000:1000], as speed command is percentage, just multiply it by 10.
    lx_servo_or_motor_mode_write(&lx_, true, command * 10);

    bool _mode;
    int16_t _speed;
    lx_servo_or_motor_mode_read(&lx_, &_mode, &_speed);

    std::cout << "LxMotor " << id_ << ": mode=" << _mode << "   speed=" << _speed << std::endl;
}

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
