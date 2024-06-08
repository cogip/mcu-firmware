// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_sensors.hpp"
#include "pf_bool_sensors.hpp"

#include "BoolSensor.hpp"

#include "etl/pool.h"
#include "etl/map.h"

namespace cogip {
namespace pf {
namespace sensors {

void init() {
    bool_sensors::init();
}

} // namespace sensors
} // namespace pf
} // namespace cogip