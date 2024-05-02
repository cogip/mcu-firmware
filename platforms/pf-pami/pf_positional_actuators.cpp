// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_positional_actuators.hpp"

#include "AnalogServo.hpp"
#include "LxMotor.hpp"
#include "PositionalActuator.hpp"

#include "pca9685_params.hpp"
#include "platform.hpp"

#include "etl/map.h"
#include "etl/pool.h"

// RIOT includes
#include <event.h>
#include <ztimer.h>
#include <motor_driver.h>

// System includes
#include <iostream>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

// Positional actuator protobuf message
static PB_PositionalActuator _pb_positional_actuator;

/// Positional actuator timeout thread stack
static char _positional_actuators_timeout_thread_stack[THREAD_STACKSIZE_DEFAULT];
/// Positional actuators timeout thread period (ms)
constexpr uint16_t _positional_actuators_timeout_thread_period_ms = 100;

/// Analog servomotor pool
static etl::pool<AnalogServo, COUNT> _analog_servo_pool;
/// Positional actuators map
static etl::map<Enum, PositionalActuator *, 4*COUNT> _positional_actuators;


/// Init I2C PWM driver
static void _pca9685_init() {
    // Init PCA9685
    if (pca9685_init(&AnalogServo::pca9685_dev, &pca9685_params) != PCA9685_OK) {
        puts("Error: PCA9685 init failed!");
        return;
    }
    if (pca9685_pwm_init(&AnalogServo::pca9685_dev, pca9685_params.mode, pca9685_params.freq, pca9685_params.res) != pca9685_params.freq) {
        puts("Error: PCA9685 PWM init failed!");
        return;
    }
}
void disable_all() {
    for (auto & iterator: _positional_actuators) {
        PositionalActuator *positional_actuator = iterator.second;
        positional_actuator->disable();
    }
}

static void *_positional_actuators_timeout_thread(void *args)
{
    (void)args;

    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        for (auto & iterator: _positional_actuators) {
            PositionalActuator *positional_actuator = iterator.second;
            if (positional_actuator->timeout_period()) {
                if (!positional_actuator->decrement_timeout_period()) {
                    positional_actuator->disable();
                }
            }
        }

        // Wait thread period to end
        ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, _positional_actuators_timeout_thread_period_ms);
    }

    return 0;
}

void init(uart_half_duplex_t *lx_stream) {
    (void) lx_stream;
    // Init PWM I2C driver
    _pca9685_init();

    // AnalogServo init
    _positional_actuators[Enum::ANALOGSERVO_PAMI] = _analog_servo_pool.create(
        Enum::ANALOGSERVO_PAMI,
        GroupEnum::NO_GROUP,
        0,
        0,
        PCA9586Channels::CHANNEL_ANALOGSERVO_PAMI
    );
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_PAMI])->add_position(analog_servomotor_pami_closed);
    static_cast<AnalogServo*>(_positional_actuators[Enum::ANALOGSERVO_PAMI])->add_position(analog_servomotor_pami_deployed);

    // Positional actuators timeout thread
    thread_create(
        _positional_actuators_timeout_thread_stack,
        sizeof(_positional_actuators_timeout_thread_stack),
        THREAD_PRIORITY_MAIN - 1,
        THREAD_CREATE_STACKTEST,
        _positional_actuators_timeout_thread,
        NULL,
        "Positional actuators timeout thread"
    );
}

bool contains(Enum id) {
    return _positional_actuators.contains(id);
}

PositionalActuator & get(Enum id) {
    return *_positional_actuators[id];
}

void send_emergency_button_pressed() {
    // Protobuf UART interface
    static cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();

    // Send protobuf message
    if (!uartpb.send_message(emergency_button_pressed_uuid)) {
        std::cerr << "Error: emergency_button_pressed_uuid message not sent" << std::endl;
    }
}

void send_emergency_button_released() {
    // Protobuf UART interface
    static cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();

    // Send protobuf message
    if (!uartpb.send_message(emergency_button_released_uuid)) {
        std::cerr << "Error: emergency_button_released_uuid message not sent" << std::endl;
    }
}

void send_state(Enum positional_actuator) {
    // Protobuf UART interface
    static cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();

    // Send protobuf message
    _pb_positional_actuator.clear();
    positional_actuators::get(positional_actuator).pb_copy(_pb_positional_actuator);
    if (!uartpb.send_message(actuator_state_uuid, &_pb_positional_actuator)) {
        std::cerr << "Error: actuator_state_uuid message not sent" << std::endl;
    }
}

void pb_copy(PB_Message & pb_message) {
    // cppcheck-suppress unusedVariable
    for (auto const & [id, actuator] : _positional_actuators) {
        actuator->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace actuators
} // namespace actuators
} // namespace pf
} // namespace cogip
