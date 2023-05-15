// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       Generic definitions related to actuators.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include <cstdint>

namespace cogip {
namespace pf {
namespace actuators {

/// Limit switches
/// @{
constexpr int pin_limit_switch_central_lift_top = GPIO_PIN(PORT_B, 14);
constexpr int pin_limit_switch_central_lift_bottom = GPIO_PIN(PORT_B, 13);
constexpr int pin_limit_switch_right_arm_top = PCF857X_GPIO_PIN(PCF857X_PORT_0, 10);
constexpr int pin_limit_switch_right_arm_bottom = PCF857X_GPIO_PIN(PCF857X_PORT_0, 11);
constexpr int pin_limit_switch_left_arm_top = PCF857X_GPIO_PIN(PCF857X_PORT_0, 12);
constexpr int pin_limit_switch_left_arm_bottom = PCF857X_GPIO_PIN(PCF857X_PORT_0, 13);
constexpr int pin_limit_switch_recal_right = PCF857X_GPIO_PIN(PCF857X_PORT_0, 14);
constexpr int pin_limit_switch_recal_left = PCF857X_GPIO_PIN(PCF857X_PORT_0, 15);
/// @}

/// LED strip
constexpr int pin_led_strip = PCF857X_GPIO_PIN(PCF857X_PORT_0, 7);

/// Enum using to group actuators
enum class GroupEnum: uint8_t {
    NO_GROUP = 0,
    CENTRAL_LIFT = 1,
    CONVEYOR_LAUNCHER = 2
};

/// Initialize all actuators
void init();

} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
