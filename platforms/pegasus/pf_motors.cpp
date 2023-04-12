// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_motors.hpp"
#include "pf_actuators.hpp"

#include "platform.hpp"

#include "etl/map.h"
#include "etl/pool.h"

#include <ztimer.h>
#include <motor_driver.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace motors {

static etl::pool<Motor, COUNT> _motors_pool;
static etl::map<Enum, Motor *, COUNT> _motors;

void init(void) {
    motor_driver_init(MOTOR_DRIVER_DEV(1));
    _motors[Enum::CENTRAL_LIFT_MOTOR] = _motors_pool.create(Enum::CENTRAL_LIFT_MOTOR, GroupEnum::CENTRAL_LIFT, 0);
    _motors[Enum::CONVEYOR_LAUNCHER_MOTOR] = _motors_pool.create(Enum::CONVEYOR_LAUNCHER_MOTOR, GroupEnum::CONVEYOR_LAUNCHER, 0);
}

Motor & get(Enum id) {
    return *_motors[id];
}

void pb_copy(PB_Message & pb_message) {
    // cppcheck-suppress unusedVariable
    for (auto const & [id, motor] : _motors) {
        motor->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace motors
} // namespace actuators
} // namespace pf
} // namespace cogip
