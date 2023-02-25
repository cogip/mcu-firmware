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
namespace motors {

std::ostream& operator << (std::ostream& os, Enum id) {
    os << static_cast<std::underlying_type_t<Enum>>(id);
    return os;
}

Motor::Motor(Enum id, GroupEnum group, uint8_t order) : Actuator(group, order), id_(id), direction_(0), speed_(0) {
    deactivate();
};

void Motor::move(bool direction, uint32_t speed) {
    auto motor_id = static_cast<std::underlying_type_t<Enum>>(id_);
    if ((speed > 100) && (motor_id < motor_driver_config[MOTOR_DRIVER_DEV(1)].nb_motors)) {
        return;
    }

    int32_t pwm_duty_cycle =
        (direction == 0 ? 1 : -1) * (int32_t)(motor_driver_config[MOTOR_DRIVER_DEV(1)].pwm_resolution * speed ) / 100;

    if (motor_driver_config[MOTOR_DRIVER_DEV(1)].motors[motor_id].pwm_channel > 0) {
        motor_set(MOTOR_DRIVER_DEV(1), motor_id, pwm_duty_cycle);
    }
    motor_enable(MOTOR_DRIVER_DEV(1), motor_id);

    activated_ = true;
    direction_ = direction;
    speed_ = speed;
}

void Motor::deactivate() {
    auto motor_id = static_cast<std::underlying_type_t<Enum>>(id_);
    motor_disable(MOTOR_DRIVER_DEV(1), motor_id);
    activated_ = false;
}

void Motor::pb_copy(PB_Motor & pb_motor) const {
    pb_motor.set_group(static_cast<PB_ActuatorsGroupEnum>(group_));
    pb_motor.set_order(order_);
    pb_motor.set_id(static_cast<PB_MotorEnum>(id_));
    pb_motor.set_activated(activated_);
    pb_motor.set_direction(direction_);
    pb_motor.set_speed(speed_);
}

} // namespace motors
} // namespace actuators
} // namespace pf
} // namespace cogip
