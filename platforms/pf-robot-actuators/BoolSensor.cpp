// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "board.h"
#include "BoolSensor.hpp"

#include <iostream>

namespace cogip {
namespace pf {
namespace sensors {
namespace bool_sensors {

std::ostream& operator << (std::ostream& os, cogip::pf::sensors::Enum id)
{
    os << static_cast<std::underlying_type_t<Enum>>(id);
    return os;
}

void BoolSensor::pb_copy(PB_BoolSensor & pb_bool_sensor) const
{
    pb_bool_sensor.set_id(static_cast<PB_BoolSensorEnum>(id_));
    pb_bool_sensor.set_state(state_);
}

} // namespace bool_sensors
} // namespace sensors
} // namespace pf
} // namespace cogip
