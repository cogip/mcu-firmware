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

/// Enable all actuators
void enable_all() {
    positional_actuators::enable_all();
}

/// Disable all actuators
void disable_all() {
    positional_actuators::disable_all();
}

/// Handle Protobuf actuator command message.
static void _handle_command(cogip::canpb::ReadBuffer & buffer)
{
    static PB_ActuatorCommand pb_command;
    pb_command.clear();
    pb_command.deserialize(buffer);
    if (pb_command.has_positional_actuator()) {
        const PB_PositionalActuatorCommand & pb_positional_actuator_command = pb_command.get_positional_actuator();
        cogip::actuators::Enum id = cogip::actuators::Enum{(uint8_t)pb_positional_actuator_command.id()};
        if (positional_actuators::contains(id)) {
            // Negative timeout is not possible
            int32_t timeout_ms = pb_positional_actuator_command.timeout();
            if (timeout_ms < 0) {
                std::cerr << "Negative timeout, do not actuate actuator with ID="
                          << static_cast<uint8_t>(id)
                          << std::endl;
                return;
            }

            // Timeout is valid, speed can be set
            positional_actuators::get(id).set_target_speed_percent(pb_positional_actuator_command.speed());
            if (timeout_ms > 0) {
                positional_actuators::get(id).actuate_timeout(
                    pb_positional_actuator_command.command(),
                    static_cast<uint32_t>(timeout_ms)
                );
            }
            else {
                positional_actuators::get(id).actuate(pb_positional_actuator_command.command());
            }
        }
    }
}

/// Actuators initialization message handler
static void _handle_actuators_init([[maybe_unused]] cogip::canpb::ReadBuffer & buffer)
{
    positional_actuators::init_sequence();
}

void init() {
    positional_actuators::init();

    cogip::canpb::CanProtobuf & canpb = pf_get_canpb();
    canpb.register_message_handler(
        command_uuid,
        canpb::message_handler_t::create<_handle_command>()
    );
    canpb.register_message_handler(
        init_uuid,
        canpb::message_handler_t::create<_handle_actuators_init>()
    );
}

} // namespace actuators
} // namespace pf
} // namespace cogip
