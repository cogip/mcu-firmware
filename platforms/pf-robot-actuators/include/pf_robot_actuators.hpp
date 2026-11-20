///  Copyright (C) 2026 COGIP Robotics association
///
///  This file is subject to the terms and conditions of the GNU Lesser
///  General Public License v2.1. See the file LICENSE in the top level
///  directory for more details.

///
///  @file
///  @defgroup    pf_robot_actuators Robot Actuators Platform
///  @ingroup     platforms
///  @brief       Actuators definition
///
///  @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
///  @{

#pragma once

// Standard includes
#include <periph/gpio.h>

#include "board.h"

namespace cogip {
namespace pf {
namespace robot_actuators {

// Pump pins
constexpr gpio_t PUMP_BOTTOM_PIN = GPIO_PIN(PORT_B, 1);
constexpr gpio_t PUMP_MIDDLE_PIN = GPIO_PIN(PORT_C, 5);
constexpr gpio_t PUMP_TOP_PIN = GPIO_PIN(PORT_B, 15);

// EV pins
constexpr gpio_t EV_BOTTOM_PIN = GPIO_PIN(PORT_A, 10);
constexpr gpio_t EV_MIDDLE_PIN = GPIO_PIN(PORT_B, 6);
constexpr gpio_t EV_TOP_PIN = GPIO_PIN(PORT_B, 5);

/// @brief Initialize robot actuators platform
void pf_init_robot_actuators(void);

/// @brief Disable all actuators
void pf_robot_actuators_disable(void);

} // namespace robot_actuators
} // namespace pf
} // namespace cogip

/// @}
