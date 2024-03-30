// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
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
    ANALOGSERVO_BOTTOM_ARM_LEFT = 2,
    ANALOGSERVO_BOTTOM_ARM_RIGHT = 3,
    ANALOGSERVO_TOP_ARM_LEFT = 4,
    ANALOGSERVO_TOP_ARM_RIGHT = 5,
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

using PB_Message = EmbeddedProto::RepeatedFieldFixedSize<PB_PositionalActuator, COUNT>;

/// PCA9586 channels
enum PCA9586Channels {
    CHANNEL_ANALOGSERVO_BOTTOM_ARM_LEFT = 0,
    CHANNEL_ANALOGSERVO_BOTTOM_ARM_RIGHT = 1,
    CHANNEL_ANALOGSERVO_TOP_ARM_LEFT = 2,
    CHANNEL_ANALOGSERVO_TOP_ARM_RIGHT = 3,
};

/// Limit switches
/// @{
constexpr gpio_t pin_limit_switch_bottom_lift = GPIO_PIN(PORT_B, 14);
constexpr gpio_t pin_limit_switch_top_lift = GPIO_PIN(PORT_B, 13);
/// @}

/// Front arms servomotor positions
/// @{
constexpr int analog_servomotor_bottom_arm_left_closed = 55;
constexpr int analog_servomotor_bottom_arm_left_opened = 135;
constexpr int analog_servomotor_bottom_arm_right_closed = 55;
constexpr int analog_servomotor_bottom_arm_right_opened = 135;
constexpr int analog_servomotor_top_arm_left_closed = 55;
constexpr int analog_servomotor_top_arm_left_opened = 135;
constexpr int analog_servomotor_top_arm_right_closed = 55;
constexpr int analog_servomotor_top_arm_right_opened = 135;
/// @}

/// Actuators timeouts
/// @{
constexpr uint32_t default_timeout_period_motor_bottom_lift = 3;
constexpr uint32_t default_timeout_period_motor_top_lift = 3;
/// @}


/// Actuators DC motors IDs
/// @{
/// @}

/// Initialize positional_actuators.
void init(uart_half_duplex_t *lx_stream);

/// GPIO expander wrapper
void pf_pcf857x_gpio_write(gpio_t pin, int value);

/// Get a positional_actuator by id.
PositionalActuator & get(
    Enum id  ///< [in] positional_actuator id
);

/// Disable all positional actuators
void disable_all();

/// Send emergency button pressed protobuf message
void send_emergency_button_pressed();

/// Send emergency button released protobuf message
void send_emergency_button_released();

/// Send positional actuator state protobuf message
void send_state(Enum positional_actuator);

/// Copy data to Protobuf message.
void pb_copy(
    PB_Message & pb_message  ///< [out] Protobuf message to fill
);

} // namespace positional_actuators
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
