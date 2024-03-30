// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
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

// Pump protobuf message
static PB_Pump _pb_pump;

void init(void) {
    // Right arm
    _pumps[Enum::RIGHT_ARM_PUMP] = _pumps_pool.create(
        Enum::RIGHT_ARM_PUMP,
        GroupEnum::NO_GROUP,
        0
    );

    // Left arm
    _pumps[Enum::LEFT_ARM_PUMP] = _pumps_pool.create(
        Enum::LEFT_ARM_PUMP,
        GroupEnum::NO_GROUP,
        0
    );
}

Pump & get(Enum id) {
    return *_pumps[id];
}

void disable_all() {
    for (auto & iterator: _pumps) {
        Pump *pump = iterator.second;
        pump->deactivate();
    }
}

void send_state(Enum pump) {
    // Protobuf UART interface
    static cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();

    // Send protobuf message
    _pb_pump.clear();
    pumps::get(pump).pb_copy(_pb_pump);
    if (!uartpb.send_message(actuator_state_uuid, &_pb_pump)) {
        std::cerr << "Error: actuator_state_uuid message not sent" << std::endl;
    }
}

void pb_copy(PB_Message & pb_message) {
    // cppcheck-suppress unusedVariable
    for (auto const & [id, pump] : _pumps) {
        pump->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace pumps
} // namespace actuators
} // namespace pf
} // namespace cogip
