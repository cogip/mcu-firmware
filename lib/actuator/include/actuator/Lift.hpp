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

// System includes
#include <climits>

// Project includes
#include "actuator/LiftParameters.hpp"
#include "actuator/Motor.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief  Vertical lift actuator built on top of the generic Motor class.
/// @details
///   Uses a static configuration (LiftParameters) to control upward and
///   downward motion, handle limit switches, and provide stop functionality
///   specific to a robot elevator.
class Lift : public Motor
{
  public:
    /// @brief Construct a Lift actuator from its configuration parameters.
    /// @param params Reference to a static LiftParameters instance containing
    ///               both MotorParameters and lift-specific speed settings.
    explicit Lift(const LiftParameters& params);

    /// @brief Perform any required initialization (e.g., homing or zeroing).
    void init();

    /// @brief Immediately brake the lift actuator.
    void stop();

    /// @brief Override of Motor::actuate.
    /// @param command Desired movement command in millimeters.
    void actuate(int32_t command) override;

    /// @brief Handle limit-switch trigger events.
    /// @param pin GPIO pin identifier for the triggered limit switch.
    void at_limits(gpio_t pin);

  protected:
    /// @brief Reset last command on blocked/timeout so the same target can be retried.
    void on_state_change(motion_control::target_pose_status_t state) override;

  private:
    /// Reference to the static configuration parameters for this lift actuator.
    const LiftParameters& params_;

    /// True while the homing sequence is running (init only).
    bool initializing_ = false;

    /// Last commanded position (after clamping), used to skip redundant actuate calls.
    int32_t last_command_ = INT32_MIN;

    /// @brief Handle action when the lower end-stop is active.
    void at_lower_limit();

    /// @brief Handle action when the upper end-stop is active.
    void at_upper_limit();
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
