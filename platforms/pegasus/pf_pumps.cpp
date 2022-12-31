// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_pumps.hpp"
#include "pf_actuators.hpp"

#include "platform.hpp"

#include "etl/map.h"
#include "etl/pool.h"

#include <ztimer.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace pumps {

static etl::pool<Pump, COUNT> _pumps_pool;
static etl::map<Enum, Pump *, COUNT> _pumps;

void init(void) {
    _pumps[Enum::ARM_CENTRAL_PUMP] = _pumps_pool.create(Enum::ARM_CENTRAL_PUMP, GroupEnum::CENTRAL_ARM, 4);
    _pumps[Enum::ARM_LEFT_PUMP] = _pumps_pool.create(Enum::ARM_LEFT_PUMP, GroupEnum::LEFT_ARM, 2);
    _pumps[Enum::ARM_RIGHT_PUMP] = _pumps_pool.create(Enum::ARM_RIGHT_PUMP, GroupEnum::RIGHT_ARM, 2);
}

Pump & get(Enum id) {
    return *_pumps[id];
}

void pb_copy(PB_Message & pb_message) {
    for (auto const & [id, pump] : _pumps) {
        pump->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace pumps
} // namespace actuators
} // namespace pf
} // namespace cogip
