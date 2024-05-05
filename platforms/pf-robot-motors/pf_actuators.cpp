// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#include "board.h"
#include "platform.hpp"

#include "canpb/CanProtobuf.hpp"
#include "canpb/ReadBuffer.hpp"
#include "thread/thread.hpp"

#include "PB_Actuators.hpp"

namespace cogip {
namespace pf {
namespace actuators {

static  bool _suspend_actuators = false;

/// Enable all actuators
void enable_all() {
    _suspend_actuators = false;
}

/// Disable all actuators
void disable_all() {
    positional_actuators::disable_all();
    _suspend_actuators = true;
}

/// Handle Protobuf actuator command message.
static void _handle_command(cogip::canpb::ReadBuffer & buffer)
{
    static PB_ActuatorCommand pb_command;
    pb_command.clear();
    pb_command.deserialize(buffer);
    if (!_suspend_actuators) {
        if (pb_command.has_positional_actuator()) {
            const PB_PositionalActuatorCommand & pb_positional_actuator_command = pb_command.get_positional_actuator();
            Enum id = Enum{(uint8_t)pb_positional_actuator_command.id()};
            if (positional_actuators::contains(id)) {
                positional_actuators::get(id).actuate(pb_positional_actuator_command.command());
            }
        }
    }
}

void init() {
    positional_actuators::init();

    cogip::canpb::CanProtobuf & canpb = pf_get_canpb();
    canpb.register_message_handler(
        command_uuid,
        canpb::message_handler_t::create<_handle_command>()
    );
}

} // namespace actuators
} // namespace pf
} // namespace cogip
