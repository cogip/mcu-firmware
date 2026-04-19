// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"

#include "pf_common/platform_common.hpp"
#include "pf_positional_actuators.hpp"

#include "board.h"
#include "log.h"
#include <inttypes.h>

#include "canpb/CanProtobuf.hpp"
#include "canpb/ReadBuffer.hpp"
#include "thread/thread.hpp"

#include "PB_Actuators.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {
namespace pf {
namespace actuators {

/// Enable all actuators
void enable_all()
{
    positional_actuators::enable_all();
}

/// Disable all actuators
void disable_all()
{
    positional_actuators::disable_all();
}

/// Handle Protobuf actuator command message.
static void _handle_command(cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("Actuator command rejected: emergency stop latched\n");
        return;
    }
    LOG_INFO("_handle_command: received CAN message\n");
    static PB_ActuatorCommand pb_command;
    pb_command.clear();
    pb_command.deserialize(buffer);
    LOG_INFO("_handle_command: has_positional_actuator=%d\n", pb_command.has_positional_actuator());
    if (pb_command.has_positional_actuator()) {
        const PB_PositionalActuatorCommand& pb_positional_actuator_command =
            pb_command.get_positional_actuator();
        cogip::actuators::Enum id =
            cogip::actuators::Enum{(uint8_t)pb_positional_actuator_command.id()};
        LOG_INFO("_handle_command: actuator id=%" PRIu8 ", contains=%d\n", static_cast<uint8_t>(id),
                 positional_actuators::contains(id));
        if (positional_actuators::contains(id)) {
            // Negative timeout is not possible
            int32_t timeout_ms = pb_positional_actuator_command.timeout();
            if (timeout_ms < 0) {
                LOG_ERROR("Negative timeout, do not actuate actuator with ID=%" PRIu8,
                          static_cast<uint8_t>(id));
                return;
            }

            // Timeout is valid, speed can be set
            positional_actuators::get(id).set_target_speed_percent(
                pb_positional_actuator_command.speed());

            if (timeout_ms > 0) {
                positional_actuators::get(id).actuate_timeout(
                    pb_positional_actuator_command.command(), static_cast<uint32_t>(timeout_ms));
            } else {
                positional_actuators::get(id).actuate(pb_positional_actuator_command.command());
            }

            LOG_INFO("Target distance: %" PRIi32, pb_positional_actuator_command.command());
            LOG_INFO("Target speed: %" PRIi32, pb_positional_actuator_command.speed());
            LOG_INFO("Timeout: %" PRIi32, timeout_ms);
        }
    }
}

/// Actuators initialization message handler.
/// Without a PB_ActuatorInit payload, or with one whose id field is
/// unset, every positional actuator is initialised (legacy behaviour).
/// When the id field is set, only that actuator is initialised.
static void _handle_actuators_init(cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf_common::clear_emergency_stop();
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("Actuator init rejected: emergency stop button still engaged\n");
        return;
    }

    static PB_ActuatorInit pb_init;
    pb_init.clear();
    if (buffer.get_size() > 0) {
        pb_init.deserialize(buffer);
    }

    if (!pb_init.has_id()) {
        positional_actuators::init_sequence();
        return;
    }

    cogip::actuators::Enum id = cogip::actuators::Enum{static_cast<uint8_t>(pb_init.get_id())};
    if (!positional_actuators::contains(id)) {
        LOG_WARNING("Actuator init: id=%" PRIu8 " not found\n", static_cast<uint8_t>(id));
        return;
    }
    LOG_INFO("Actuator init: id=%" PRIu8 "\n", static_cast<uint8_t>(id));
    positional_actuators::get(id).init();
}

void init()
{
    positional_actuators::init();

    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();
    canpb.register_message_handler(command_uuid,
                                   canpb::message_handler_t::create<_handle_command>());
    canpb.register_message_handler(init_uuid,
                                   canpb::message_handler_t::create<_handle_actuators_init>());
    LOG_INFO("pf_actuators::init: registered handler for command_uuid=0x%04" PRIX32 "\n",
             command_uuid);
}

} // namespace actuators
} // namespace pf
} // namespace cogip
