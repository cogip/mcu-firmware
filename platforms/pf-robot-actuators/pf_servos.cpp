// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

// Firmware includes
#include "board.h"
#include "pf_servos.hpp"
#include "pf_actuators.hpp"
#include "platform.hpp"
#include "canpb/CanProtobuf.hpp"

#include "etl/map.h"
#include "etl/pool.h"

// RIOT includes
#include <periph/gpio.h>
#include <ztimer.h>

#ifndef LX_DIR_PIN
#define LX_DIR_PIN  GPIO_UNDEF
#endif

namespace cogip {
namespace pf {
namespace actuators {
namespace servos {

/// Static pool of servo objects
static etl::pool<LxServo, COUNT> _servos_pool;

/// Map from servo id to servo object pointer
static etl::map<Enum, LxServo *, COUNT> _servos;

// Actuator state protobuf message
static PB_ActuatorState _pb_actuator_state;

void init(uart_half_duplex_t *lx_stream) {
    // Half duplex stream that must have been initialized previously
    LxServo::lx_stream = lx_stream;

    // Left arm
    _servos[Enum::LXSERVO_LEFT_CART] = _servos_pool.create(
        Enum::LXSERVO_LEFT_CART
    );

    // Right arm
    _servos[Enum::LXSERVO_RIGHT_CART] = _servos_pool.create(
        Enum::LXSERVO_RIGHT_CART
    );

    // Panel arm
    _servos[Enum::LXSERVO_ARM_PANEL] = _servos_pool.create(
        Enum::LXSERVO_ARM_PANEL
    );
}

LxServo & get(Enum id) {
    return *_servos[id];
}

void move(const Command & command, uint32_t wait) {
    servos::get(command.id).move(command.position, command.time);
    if (wait > 0) {
        ztimer_sleep(ZTIMER_MSEC, wait);
    }
}

void parallel_move(const etl::list<Command, COUNT> & commands, uint32_t wait) {
    for (auto & command: commands) {
        servos::get(command.id).move_wait(command.position, command.time);
    }
    for (auto & command: commands) {
        servos::get(command.id).move_start();
    }
    if (wait > 0) {
        ztimer_sleep(ZTIMER_MSEC, wait);
    }
}

void disable_all() {
    for (auto & iterator: _servos) {
        LxServo *servo = iterator.second;
        servo->move_stop();
    }
}

void send_state(Enum servo) {
    // Protobuf CAN interface
    static cogip::canpb::CanProtobuf & canpb = pf_get_canpb();

    // Send protobuf message
    _pb_actuator_state.clear();
    servos::get(servo).pb_copy(_pb_actuator_state.mutable_servo());
    if (!canpb.send_message(actuator_state_uuid, &_pb_actuator_state)) {
        std::cerr << "Error: actuator_state_uuid message not sent" << std::endl;
    }
}

void send_states() {
    for (auto const & [id, servo] : _servos) {
        send_state(id);
    }
}

} // namespace servos
} // namespace actuators
} // namespace pf
} // namespace cogip
