// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_actuators.hpp"
#include "pf_servos.hpp"
#include "pf_positional_actuators.hpp"

#include "board.h"
#include "platform.hpp"

#include "canpb/CanProtobuf.hpp"
#include "canpb/ReadBuffer.hpp"
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
static  bool _suspend_actuators = false;

/// Half duplex CAN stream
static uart_half_duplex_t _lx_stream;

/// LX servos command buffer
static uint8_t _lx_servos_buffer[LX_UART_BUFFER_SIZE];

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

        servos::send_states();

        // Wait thread period to end
        cogip::thread::thread_ztimer_periodic_wakeup(ZTIMER_MSEC, &loop_start_time, SENDER_PERIOD_MSEC);
    }

    return NULL;
}

/// Disable all actuators
void enable_all() {
    _suspend_actuators = false;
}

/// Disable all actuators
void disable_all() {
    positional_actuators::disable_all();
    servos::disable_all();
    _suspend_actuators = true;
}

/// Start threading sending actuators state.
static void _handle_thread_start([[maybe_unused]] cogip::canpb::ReadBuffer & buffer)
{
    _suspend_sender = false;
    thread_wakeup(_sender_pid);
}

/// Stop threading sending actuators state.
static void _handle_thread_stop([[maybe_unused]] cogip::canpb::ReadBuffer & buffer)
{
    _suspend_sender = true;
}

/// Handle Protobuf actuator command message.
static void _handle_command(cogip::canpb::ReadBuffer & buffer)
{
    static PB_ActuatorCommand pb_command;
    pb_command.clear();
    pb_command.deserialize(buffer);
    if (!_suspend_actuators) {
        if (pb_command.has_servo()) {
            const PB_ServoCommand & pb_servo_command = pb_command.get_servo();
            servos::Enum id = servos::Enum{(lx_id_t)pb_servo_command.id()};
            if (servos::contains(id)) {
                servos::move(
                    {
                        id,
                        (uint16_t)pb_servo_command.command()
                    }
                );
            }
        }
        if (pb_command.has_positional_actuator()) {
            const PB_PositionalActuatorCommand & pb_positional_actuator_command = pb_command.get_positional_actuator();
            Enum id = Enum{(uint8_t)pb_positional_actuator_command.id()};
            if (positional_actuators::contains(id)) {
                positional_actuators::get(id).actuate(pb_positional_actuator_command.command());
            }
        }
    }
}

static void _dir_init([[maybe_unused]] uart_t uart) {
#ifdef LX_DIR_PIN
    gpio_init(LX_DIR_PIN, GPIO_OUT);
#endif
}

static void _dir_enable_tx([[maybe_unused]] uart_t uart) {
#ifdef LX_DIR_PIN
    gpio_clear(LX_DIR_PIN);
#endif
}

static void _dir_disable_tx([[maybe_unused]] uart_t uart) {
#ifdef LX_DIR_PIN
    gpio_set(LX_DIR_PIN);
#endif
}

static void _lx_half_duplex_uart_init() {
    uart_half_duplex_params_t params = {
        .uart = UART_DEV(LX_UART_DEV),
        .baudrate = 115200,
        .dir = { _dir_init, _dir_enable_tx, _dir_disable_tx },
    };

    int ret = uart_half_duplex_init(&_lx_stream, _lx_servos_buffer, ARRAY_SIZE(_lx_servos_buffer), &params);

    if (ret == UART_HALF_DUPLEX_NODEV) {
        puts("Error: invalid CAN device given");
    }
    else if (ret == UART_HALF_DUPLEX_NOBAUD) {
        puts("Error: given baudrate is not applicable");
    }
    else if (ret == UART_HALF_DUPLEX_INTERR) {
        puts("Error: internal error");
    }
    else if (ret == UART_HALF_DUPLEX_NOMODE) {
        puts("Error: given mode is not applicable");
    }
    else if (ret == UART_HALF_DUPLEX_NOBUFF) {
        puts("Error: invalid buffer given");
    }
    else {
        printf("Successfully initialized LX Servos TTL bus UART_DEV(%i)\n", params.uart);
    }
}

void init() {
    _lx_half_duplex_uart_init();
    positional_actuators::init();
    servos::init(&_lx_stream);

    _sender_pid = thread_create(
        _sender_stack,
        sizeof(_sender_stack),
        SENDER_PRIO,
        THREAD_CREATE_STACKTEST | THREAD_CREATE_SLEEPING,
        _thread_sender,
        NULL,
        "Actuators state"
        );

    cogip::canpb::CanProtobuf & canpb = pf_get_canpb();
    canpb.register_message_handler(
        command_uuid,
        canpb::message_handler_t::create<_handle_command>()
    );
}

} // namespace actuators
} // namespace pf
} // namespace cogip
