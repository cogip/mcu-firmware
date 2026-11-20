///  Copyright (C) 2026 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///  @ingroup     platforms
///  @brief       robot actuators platform
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

// Standard includes
#include <cstddef>
#include <inttypes.h>

// RIOT includes
#include "log.h"

// Platform includes
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"
#include "pf_robot_actuators.hpp"

// Libraries includes
#include "gpio/gpio.hpp"

// CAN protobuf includes
#include "canpb/CanProtobuf.hpp"
#include "canpb/ReadBuffer.hpp"

// Import protobuf messages (generated files will be in the build directory)
#include "PB_Actuators.hpp"

namespace cogip {
namespace pf {
namespace robot_actuators {

// GPIO objects creation
static gpio::GPIO pump_bottom_gpio_(PUMP_BOTTOM_PIN, GPIO_OUT);
static gpio::GPIO pump_middle_gpio_(PUMP_MIDDLE_PIN, GPIO_OUT);
static gpio::GPIO pump_top_gpio_(PUMP_TOP_PIN, GPIO_OUT);
static gpio::GPIO ev_bottom_gpio_(EV_BOTTOM_PIN, GPIO_OUT);
static gpio::GPIO ev_middle_gpio_(EV_MIDDLE_PIN, GPIO_OUT);
static gpio::GPIO ev_top_gpio_(EV_TOP_PIN, GPIO_OUT);

/// On/off actuators GPIOs, indexed by PB_OnOffActuatorEnum
static gpio::GPIO* const _onoff_actuators[] = {
    &pump_bottom_gpio_, // PUMP_BOTTOM
    &pump_middle_gpio_, // PUMP_MIDDLE
    &pump_top_gpio_,    // PUMP_TOP
    &ev_bottom_gpio_,   // EV_BOTTOM
    &ev_middle_gpio_,   // EV_MIDDLE
    &ev_top_gpio_,      // EV_TOP
};

/// Number of on/off actuators
static constexpr size_t _onoff_actuators_numof =
    sizeof(_onoff_actuators) / sizeof(_onoff_actuators[0]);

/// Handle Protobuf actuator command message.
static void _handle_command(cogip::canpb::ReadBuffer& buffer)
{
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("Actuator command rejected: emergency stop latched\n");
        return;
    }

    static PB_ActuatorCommand pb_command;
    pb_command.clear();
    pb_command.deserialize(buffer);

    if (!pb_command.has_onoff_actuator()) {
        LOG_WARNING("Actuator command rejected: invalid command\n");
        return;
    }

    const PB_OnOffActuatorCommand& pb_onoff_command = pb_command.get_onoff_actuator();
    const size_t id = static_cast<size_t>(pb_onoff_command.id());
    if (id >= _onoff_actuators_numof) {
        LOG_WARNING("Actuator command: on/off id=%u not found\n", static_cast<unsigned>(id));
        return;
    }

    if (pb_onoff_command.command()) {
        _onoff_actuators[id]->set();
    } else {
        _onoff_actuators[id]->clear();
    }
}

/// Actuators initialization message handler.
static void _handle_init([[maybe_unused]] cogip::canpb::ReadBuffer& buffer)
{
    cogip::pf_common::clear_emergency_stop();
    if (cogip::pf_common::is_emergency_stop_latched()) {
        LOG_WARNING("Actuator init rejected: emergency stop button still engaged\n");
        return;
    }

    pf_robot_actuators_disable();
}

void pf_init_robot_actuators(void)
{
    pf_robot_actuators_disable();

    cogip::canpb::CanProtobuf& canpb = cogip::pf_common::get_canpb();
    canpb.register_message_handler(pf_common::actuator_command_uuid,
                                   canpb::message_handler_t::create<_handle_command>());
    canpb.register_message_handler(pf_common::actuator_init_uuid,
                                   canpb::message_handler_t::create<_handle_init>());
}

void pf_robot_actuators_disable(void)
{
    for (gpio::GPIO* gpio : _onoff_actuators) {
        gpio->clear();
    }
}

} // namespace robot_actuators
} // namespace pf
} // namespace cogip

/// @}
