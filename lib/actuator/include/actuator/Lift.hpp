// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       Lift actuator class definition.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "actuator/LiftParameters.hpp"
#include "actuator/Motor.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief  Vertical lift actuator built on top of the generic Motor class.
/// @details
///   Uses a static configuration (LiftParameters) to control upward and downward motion,
///   handle limit switches, and provide stop functionality specific to a robot elevator.
class Lift : public Motor {
public:
    /// @brief Construct a Lift actuator from its configuration parameters.
    /// @param params Reference to a static LiftParameters instance containing
    ///               both MotorParameters and lift-specific speed settings.
    explicit Lift(const LiftParameters& params);

    /// @brief Perform any required initialization (e.g., homing or zeroing).
    void init();

    /// @brief Immediately brake the lift actuator.
    void stop();

    /// @brief Override of Motor::actuate with limit-switch enforcement.
    /// @param command Desired movement command in millimeters.
    void actuate(int32_t command) override;

    /// @brief Handle limit-switch trigger events.
    /// @param pin GPIO pin identifier for the triggered limit switch.
    void at_limits(gpio_t pin);

private:
    /// Reference to the static configuration parameters for this lift actuator.
    const LiftParameters& params_;

    /// @brief Handle action when the lower end-stop is active.
    void at_lower_limit();

    /// @brief Handle action when the upper end-stop is active.
    void at_upper_limit();
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
