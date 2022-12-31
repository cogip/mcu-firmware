// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "pf_servos.hpp"
#include "pf_actuators.hpp"

#include "board.h"

#include "etl/map.h"
#include "etl/pool.h"

#include <periph/gpio.h>
#include <ztimer.h>

#ifndef LX_DIR_PIN
#define LX_DIR_PIN  GPIO_UNDEF
#endif

namespace cogip {
namespace pf {
namespace actuators {
namespace servos {

/// Half duplex UART stream
static uart_half_duplex_t _lx_stream;

/// LX servos command buffer
static uint8_t _lx_servos_buffer[LX_UART_BUFFER_SIZE];

/// Static pool of servo objects
static etl::pool<LxServo, COUNT> _servos_pool;

/// Map from servo id to servo object pointer
static etl::map<Enum, LxServo *, COUNT> _servos;

static void _dir_init([[maybe_unused]] uart_t uart) {
    gpio_init(LX_DIR_PIN, GPIO_OUT);
}

static void _dir_enable_tx([[maybe_unused]] uart_t uart) {
    gpio_set(LX_DIR_PIN);
}

static void _dir_disable_tx([[maybe_unused]] uart_t uart) {
    gpio_clear(LX_DIR_PIN);
}

static void _lx_half_duplex_uart_init() {
    uart_half_duplex_params_t params = {
        .uart = UART_DEV(LX_UART_DEV),
        .baudrate = 115200,
        .dir = { _dir_init, _dir_enable_tx, _dir_disable_tx },
    };

    int ret = uart_half_duplex_init(&_lx_stream, _lx_servos_buffer, ARRAY_SIZE(_lx_servos_buffer), &params);

    if (ret == UART_HALF_DUPLEX_NODEV) {
        puts("Error: invalid UART device given");
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

void init(void) {
    _lx_half_duplex_uart_init();
    LxServo::lx_stream = &_lx_stream;
    _servos[Enum::ARM_CENTRAL_LIFT] = _servos_pool.create(Enum::ARM_CENTRAL_LIFT, GroupEnum::CENTRAL_ARM, 0);
    _servos[Enum::ARM_CENTRAL_BASE] = _servos_pool.create(Enum::ARM_CENTRAL_BASE, GroupEnum::CENTRAL_ARM, 1);
    _servos[Enum::ARM_CENTRAL_MID] = _servos_pool.create(Enum::ARM_CENTRAL_MID, GroupEnum::CENTRAL_ARM, 2);
    _servos[Enum::ARM_CENTRAL_HEAD] = _servos_pool.create(Enum::ARM_CENTRAL_HEAD, GroupEnum::CENTRAL_ARM, 3);
    _servos[Enum::ARM_RIGHT_BASE] = _servos_pool.create(Enum::ARM_RIGHT_BASE, GroupEnum::RIGHT_ARM, 0);
    _servos[Enum::ARM_RIGHT_HEAD] = _servos_pool.create(Enum::ARM_RIGHT_HEAD, GroupEnum::RIGHT_ARM, 1);
    _servos[Enum::ARM_LEFT_BASE] = _servos_pool.create(Enum::ARM_LEFT_BASE, GroupEnum::LEFT_ARM, 0);
    _servos[Enum::ARM_LEFT_HEAD] = _servos_pool.create(Enum::ARM_LEFT_HEAD, GroupEnum::LEFT_ARM, 1);
    _servos[Enum::STORAGE] = _servos_pool.create(Enum::STORAGE, GroupEnum::NO_GROUP);
    _servos[Enum::WHEEL] = _servos_pool.create(Enum::WHEEL, GroupEnum::NO_GROUP);
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

void pb_copy(PB_Message & pb_message) {
    for (auto const & [id, servo] : _servos) {
        servo->pb_copy(pb_message.get(pb_message.get_length()));
    }
}

} // namespace servos
} // namespace actuators
} // namespace pf
} // namespace cogip
