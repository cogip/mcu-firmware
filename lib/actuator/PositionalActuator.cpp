// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "board.h"
#include "actuator/PositionalActuator.hpp"

#include <iostream>

namespace cogip {
namespace actuators {
namespace positional_actuators {

std::ostream& operator << (std::ostream& os, Enum id)
{
    os << static_cast<std::underlying_type_t<Enum>>(id);
    return os;
}

void PositionalActuator::actuate_timeout(int32_t command, uint32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;
    actuate(command);
}

void PositionalActuator::pb_copy(PB_PositionalActuator & pb_positional_actuator) const
{
    pb_positional_actuator.set_id(static_cast<PB_PositionalActuatorEnum>(id_));
    pb_positional_actuator.set_state(PB_PositionalActuatorStateEnum::REACHED);
    pb_positional_actuator.set_position(command_);
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
