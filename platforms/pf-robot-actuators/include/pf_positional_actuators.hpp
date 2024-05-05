// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       Functions and definitions related to positional_actuators.
/// @author      Eric Courtois <eric.courtois@gmail.com>
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "board.h"
#include "PositionalActuator.hpp"

// RIOT includes
#include <pca9685.h>
#include <periph/gpio.h>
#include <uart_half_duplex.h>

#ifndef PCA9685_OFFSET
    #define PCA9685_OFFSET 0
#endif
namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

// Motors ids
constexpr auto START_LINE = __LINE__;
enum class Enum: uint8_t {
    MOTOR_BOTTOM_LIFT = 0,
    MOTOR_TOP_LIFT = 1,
    ANALOGSERVO_BOTTOM_GRIP_LEFT = 2,
    ANALOGSERVO_BOTTOM_GRIP_RIGHT = 3,
    ANALOGSERVO_TOP_GRIP_LEFT = 4,
    ANALOGSERVO_TOP_GRIP_RIGHT = 5,
    CART_MAGNET_LEFT = 6,
    CART_MAGNET_RIGHT = 7,
    /* ANALOGSERVO_PAMI8 = 8 */
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

/// PCA9586 channels
enum PCA9586Channels {
    CHANNEL_ANALOGSERVO_TOP_GRIP_RIGHT = 0,
    CHANNEL_ANALOGSERVO_TOP_GRIP_LEFT = 1,
    CHANNEL_ANALOGSERVO_BOTTOM_GRIP_RIGHT = 2,
    CHANNEL_ANALOGSERVO_BOTTOM_GRIP_LEFT = 3,
};

/// Magnet command pins
/// @{
constexpr gpio_t pin_cart_magnet_left = GPIO_PIN(PORT_C, 7);
constexpr gpio_t pin_cart_magnet_right = GPIO_PIN(PORT_C, 9);
/// @}

/// Initialize positional_actuators.
void init();

/// Check if a positional_actuator identified by id exists.
bool contains(
    cogip::pf::actuators::Enum id  ///< [in] positional_actuator id
);

/// Get a positional_actuator by id.
PositionalActuator & get(
    cogip::pf::actuators::Enum id  ///< [in] positional_actuator id
);

/// Disable all positional actuators
void disable_all();

/// Send positional actuator state protobuf message
void send_state(cogip::pf::actuators::Enum positional_actuator);

/// Send all positional actuator states
void send_states();

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
