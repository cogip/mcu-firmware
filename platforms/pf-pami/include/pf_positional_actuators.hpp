// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pami
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
#include <pcf857x.h>
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
    ANALOGSERVO_PAMI = 8,
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

/// PCA9586 channels
enum PCA9586Channels {
    CHANNEL_ANALOGSERVO_PAMI = 0,
};

/// Servomotor positions
/// @{
constexpr int analog_servomotor_pami_closed = 55;
constexpr int analog_servomotor_pami_deployed = 235;
/// @}

/// Initialize positional_actuators.
void init();

/// GPIO expander wrapper
void pf_pcf857x_gpio_write(gpio_t pin, int value);

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

/// Send emergency button pressed protobuf message
void send_emergency_button_pressed();

/// Send emergency button released protobuf message
void send_emergency_button_released();

/// Send positional actuator state protobuf message
void send_state(cogip::pf::actuators::Enum positional_actuator);

/// Send all positional actuator states
void send_states();

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
