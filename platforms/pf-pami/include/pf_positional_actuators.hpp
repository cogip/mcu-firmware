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
    MOTOR_CENTRAL_LIFT = 0,
    MOTOR_CONVEYOR_LAUNCHER = 1,
    ONOFF_LED_PANELS = 2,
    ANALOGSERVO_CHERRY_ARM = 3,
    ANALOGSERVO_CHERRY_ESC = 4,
    ANALOGSERVO_CHERRY_RELEASE = 5,
    LXMOTOR_RIGHT_ARM_LIFT = 6,
    LXMOTOR_LEFT_ARM_LIFT = 7
};
constexpr auto COUNT = __LINE__ - START_LINE - 3;

using PB_Message = EmbeddedProto::RepeatedFieldFixedSize<PB_PositionalActuator, COUNT>;

/// PCA9586 channels
enum PCA9586Channels {
    CHANNEL_ANALOGSERVO_CHERRY_ARM = 0,
    CHANNEL_ANALOGSERVO_CHERRY_ESC = 1,
    CHANNEL_ANALOGSERVO_CHERRY_RELEASE = 2
};

/// Limit switches
/// @{
constexpr int pin_24v_check = GPIO_PIN(PORT_C, 1);
constexpr int pin_sensor_pump_right = GPIO_PIN(PORT_A, 15);
constexpr int pin_sensor_pump_left = GPIO_PIN(PORT_C, 10);
constexpr int pin_limit_switch_central_lift_top = GPIO_PIN(PORT_B, 14);
constexpr int pin_limit_switch_central_lift_bottom = GPIO_PIN(PORT_B, 13);
constexpr int pin_limit_switch_right_arm_lift_top = PCF857X_GPIO_PIN(PCF857X_PORT_0, 10);
constexpr int pin_limit_switch_right_arm_lift_bottom = PCF857X_GPIO_PIN(PCF857X_PORT_0, 11);
constexpr int pin_limit_switch_left_arm_lift_top = PCF857X_GPIO_PIN(PCF857X_PORT_0, 12);
constexpr int pin_limit_switch_left_arm_lift_bottom = PCF857X_GPIO_PIN(PCF857X_PORT_0, 13);
constexpr int pin_limit_switch_recal_right = PCF857X_GPIO_PIN(PCF857X_PORT_0, 14);
constexpr int pin_limit_switch_recal_left = PCF857X_GPIO_PIN(PCF857X_PORT_0, 15);
/// @}

/// LED panel
constexpr int pin_led_panels = PCF857X_GPIO_PIN(PCF857X_PORT_0, 7);

/// Cherry arm servomotor positions
/// @{
constexpr int analog_servomotor_cherry_arm_closed = 55;
constexpr int analog_servomotor_cherry_arm_deployed = 235;
/// @}

/// Cherry ESC servomotor positions
/// @{
constexpr int analog_servomotor_cherry_esc_init_off = 150 + PCA9685_OFFSET;
constexpr int analog_servomotor_cherry_esc_low = 155 + PCA9685_OFFSET;
constexpr int analog_servomotor_cherry_esc_middle = 165 + PCA9685_OFFSET;
constexpr int analog_servomotor_cherry_esc_high = 175 + PCA9685_OFFSET;
constexpr int analog_servomotor_cherry_esc_max = 200 + PCA9685_OFFSET;
constexpr int analog_servomotor_cherry_esc_release = 140 + PCA9685_OFFSET;
/// @}

/// Cherry release servomotor positions
/// @{
constexpr int analog_servomotor_cherry_release_down = 180;
constexpr int analog_servomotor_cherry_release_up = 40;
/// @}

/// Actuators timeouts
/// @{
constexpr uint32_t default_timeout_period_motor_central_lift = 35;
/// @}


/// Actuators DC motors IDs
/// @{
constexpr int actuator_central_lift_motor = 0;
constexpr int actuator_conveyor_launcher_motor = 1;
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
