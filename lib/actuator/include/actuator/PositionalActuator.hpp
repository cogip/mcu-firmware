// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       C++ class representing a positional actuator.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "Actuator.hpp"
// Packages includes
#include "PB_Actuators.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief Stream insertion operator for actuator Enum ID.
/// @param os  Output stream.
/// @param id  Actuator Enum ID to insert.
/// @return    Reference to the output stream.
std::ostream& operator<<(std::ostream& os, Enum id);

/// @class PositionalActuator
/// @brief Abstract base class for positional actuators.
/// @details
///   Extends @ref Actuator with timeout management, command storage,
///   and Protobuf serialization support.
class PositionalActuator : public Actuator
{
  public:
    /// @brief Constructor.
    /// @param id                  Actuator identifier.
    /// @param default_timeout_ms  Default timeout period (in ms) before disabling
    /// on inactivity.
    /// @param send_state_cb       Optional callback to send the actuator’s state.
    PositionalActuator(Enum id, uint32_t default_timeout_ms = 0,
                       send_state_cb_t send_state_cb = nullptr)
        : Actuator(id, send_state_cb), command_(0), timeout_ms_(0),
          default_timeout_ms_(default_timeout_ms)
    {
    }

    /// @brief Get current timeout value.
    /// @return Remaining timeout in milliseconds.
    uint32_t timeout_ms() const;

    /// @brief Perform any required initialization (e.g., homing).
    virtual void init() = 0;

    /// @brief Activate the actuator with a command.
    /// @param command  Desired actuator command as a duty cycle percentage.
    virtual void actuate(int32_t command) = 0;

    /// @brief Activate the actuator for a limited time.
    /// @param command     Desired actuator command as a duty cycle percentage.
    /// @param timeout_ms  Timeout period in milliseconds.
    void actuate_timeout(int32_t command, uint32_t timeout_ms);

    /// @brief Disable the actuator immediately.
    virtual void disable() = 0;

    /// @brief Enable the actuator.
    virtual void enable() = 0;

    /// @brief Get the target speed as a percentage of maximum speed.
    /// @return Target speed in percent.
    virtual float get_target_speed_percentage() const = 0;

    /// @brief Set the target speed as a percentage of maximum speed.
    /// @param percentage  Target speed in percent.
    virtual void set_target_speed_percent(float percentage) = 0;

    /// @brief Copy actuator state into a Protobuf message.
    /// @param pb_positional_actuator  Protobuf message to populate.
    void pb_copy(PB_PositionalActuator& pb_positional_actuator) const;

  protected:
    /// Current actuator command as a duty cycle percentage.
    int32_t command_;

    /// Remaining timeout in milliseconds (decrements each thread period).
    uint32_t timeout_ms_;

    /// Default timeout to apply if none set (in milliseconds).
    uint32_t default_timeout_ms_;
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
