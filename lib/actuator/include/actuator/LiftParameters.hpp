// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       Lift actuator parameters structure definition.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// RIOT includes
#include <periph/gpio.h>

// Project includes
#include "actuator/MotorParameters.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @struct LiftParameters
/// @brief Holds configuration parameters for a Lift actuator.
/// @details
///   Combines generic motor parameters with lift-specific limits,
///   speed settings, and limit-switch pin assignments.
struct LiftParameters {
    /// Lift motor configuration parameters.
    MotorParameters motor_params;

    /// Default initialization speed as a percentage of the motor's max speed.
    float init_speed_percentage;

    /// Lower travel limit in millimeters.
    int32_t lower_limit_mm;

    /// Upper travel limit in millimeters.
    int32_t upper_limit_mm;

    /// GPIO pin connected to the lower limit switch.
    gpio_t lower_limit_switch_pin;

    /// GPIO pin connected to the upper limit switch.
    gpio_t upper_limit_switch_pin;
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
