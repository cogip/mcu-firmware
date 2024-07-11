// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

// Firmware includes
#include "board.h"
#include "pf_servos.hpp"
#include "pf_actuators.hpp"
#include "platform.hpp"
#include "uartpb/UartProtobuf.hpp"

#include "etl/map.h"
#include "etl/pool.h"

// RIOT includes
#include <periph/gpio.h>
#include <ztimer.h>

// System includes
#include <iostream>

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

// Servo protobuf message
static PB_Servo _pb_servo;

void init(uart_half_duplex_t *lx_stream) {
    // Half duplex stream that must have been initialized previously
    LxServo::lx_stream = lx_stream;

    // Ball switch
    _servos[Enum::LXSERVO_BALL_SWITCH] = _servos_pool.create(
        Enum::LXSERVO_BALL_SWITCH,
        GroupEnum::NO_GROUP,
        0
    );

    // Right arm
    _servos[Enum::LXSERVO_RIGHT_ARM] = _servos_pool.create(
        Enum::LXSERVO_RIGHT_ARM,
        GroupEnum::NO_GROUP,
        0
    );

    // Left arm
    _servos[Enum::LXSERVO_LEFT_ARM] = _servos_pool.create(
        Enum::LXSERVO_LEFT_ARM,
        GroupEnum::NO_GROUP,
        0
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
    // Protobuf UART interface
    static cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();

    // Send protobuf message
    _pb_servo.clear();
    servos::get(servo).pb_copy(_pb_servo);
    if (!uartpb.send_message(actuator_state_uuid, &_pb_servo)) {
        std::cerr << "Error: actuator_state_uuid message not sent" << std::endl;
    }
}

void pb_copy(PB_Message & pb_message) {
    // cppcheck-suppress unusedVariable
    for (auto const & [id, servo] : _servos) {
        servo->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace servos
} // namespace actuators
} // namespace pf
} // namespace cogip
