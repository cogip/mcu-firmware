// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_servos.hpp"
#include "pf_pumps.hpp"
#include "pf_motors.hpp"

#include "platform.hpp"

#include "uartpb/UartProtobuf.hpp"
#include "uartpb/ReadBuffer.hpp"
#include "thread/thread.hpp"

#include "PB_Actuators.hpp"

#define SENDER_PRIO         (THREAD_PRIORITY_MAIN)
#define SENDER_PERIOD_MSEC  (500)

namespace cogip {
namespace pf {
namespace actuators {

static kernel_pid_t _sender_pid;
static char _sender_stack[THREAD_STACKSIZE_MEDIUM];
static  bool _suspend_sender = false;

constexpr cogip::uartpb::uuid_t thread_start_uuid = 1525532810;
constexpr cogip::uartpb::uuid_t thread_stop_uuid = 3781855956;
constexpr cogip::uartpb::uuid_t state_uuid = 1538397045;
constexpr cogip::uartpb::uuid_t command_uuid = 2552455996;

static PB_ActuatorsState<cogip::pf::actuators::servos::COUNT, cogip::pf::actuators::pumps::COUNT, cogip::pf::actuators::motors::COUNT> _pb_state;

/// Build and send Protobuf actuators state message.
static void _send_state() {
    static cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();
    _pb_state.clear();
    servos::pb_copy(_pb_state.mutable_servos());
    pumps::pb_copy(_pb_state.mutable_pumps());
    motors::pb_copy(_pb_state.mutable_motors());
    uartpb.send_message(state_uuid, &_pb_state);
}

/// Actuators state sender thread.
static void *_thread_sender([[maybe_unused]] void *arg)
{
    // Init loop iteration start time
    ztimer_now_t loop_start_time = ztimer_now(ZTIMER_MSEC);

    while (true) {
        if (_suspend_sender) {
            _suspend_sender = false;
            thread_sleep();
        }

        _send_state();

        // Wait thread period to end
        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, SENDER_PERIOD_MSEC);
    }

    return NULL;
}

/// Start threading sending actuators state.
static void _handle_thread_start([[maybe_unused]] cogip::uartpb::ReadBuffer & buffer)
{
    _suspend_sender = false;
    thread_wakeup(_sender_pid);
}

/// Stop threading sending actuators state.
static void _handle_thread_stop([[maybe_unused]] cogip::uartpb::ReadBuffer & buffer)
{
    _suspend_sender = true;
}

/// Handle Protobuf actuator command message.
static void _handle_command(cogip::uartpb::ReadBuffer & buffer)
{
    static PB_ActuatorCommand pb_command;
    pb_command.clear();
    pb_command.deserialize(buffer);
    if (pb_command.has_servo()) {
        const PB_ServoCommand & pb_servo_command = pb_command.get_servo();
        servos::move(
            {
                servos::Enum{(lx_id_t)pb_servo_command.id()},
                (uint16_t)pb_servo_command.command()
            }
        );
    }
    else if (pb_command.has_pump()) {
        const PB_PumpCommand & pb_pump_command = pb_command.get_pump();
        pumps::Enum id = pumps::Enum{(vacuum_pump_t)pb_pump_command.id()};
        pumps::get(id).activate(pb_pump_command.command());
    }
    if (pb_command.has_motor()) {
        const PB_MotorCommand & pb_motor_command = pb_command.get_motor();
        motors::Enum id = motors::Enum{(uint8_t)pb_motor_command.id()};
        if (pb_motor_command.speed())
            motors::get(id).move(pb_motor_command.direction(), pb_motor_command.speed());
        else {
            motors::get(id).deactivate();
        }
    }
}

void init() {
    servos::init();
    pumps::init();
    motors::init();

    _sender_pid = thread_create(
        _sender_stack,
        sizeof(_sender_stack),
        SENDER_PRIO,
        THREAD_CREATE_STACKTEST | THREAD_CREATE_SLEEPING,
        _thread_sender,
        NULL,
        "Actuators state"
        );

    cogip::uartpb::UartProtobuf & uartpb = pf_get_uartpb();
    uartpb.register_message_handler(
        thread_start_uuid,
        uartpb::message_handler_t::create<_handle_thread_start>()
    );
    uartpb.register_message_handler(
        thread_stop_uuid,
        uartpb::message_handler_t::create<_handle_thread_stop>()
    );
    uartpb.register_message_handler(
        command_uuid,
        uartpb::message_handler_t::create<_handle_command>()
    );
}

} // namespace actuators
} // namespace pf
} // namespace cogip
