// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       Base actuators class definition.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// System includes
#include <inttypes.h>

namespace cogip {
namespace actuators {

/// @enum GroupEnum
/// Forward declaration of GroupEnum to identify actuators groups.
enum class GroupEnum : uint8_t;

/// @enum Enum
/// Forward declaration of Enum used to identify actuators.
enum class Enum : uint8_t;

/// @typedef send_state_cb_t
/// Function pointer type for sending actuator state.
///
/// This callback takes an actuator Enum ID and sends its state,
/// typically over a communication interface.
typedef void (*send_state_cb_t)(Enum id);

/// @class Actuator
/// @brief Represents a generic actuator with block state and state reporting.
///
/// Provides basic functionality:
/// - Unique identifier
/// - Blocked state management
/// - State sending via callback mechanism
class Actuator
{
  public:
    /// @brief Constructor.
    ///
    /// @param id            Unique identifier of the actuator.
    /// @param send_state_cb Optional callback to send the actuator's state.
    explicit Actuator(Enum id,                                ///< actuator id
                      send_state_cb_t send_state_cb = nullptr ///< send state callback
                      )
        : id_(id), blocked_(false), send_state_cb_(send_state_cb)
    {
    }

    /// @brief Get blocked state.
    /// @return true if blocked, false otherwise.
    bool blocked()
    {
        return blocked_;
    }

    /// @brief Set blocked state.
    /// @param blocked true to block the actuator, false to unblock.
    void set_blocked(bool blocked)
    {
        blocked_ = blocked;
    }

    /// @brief Send actuator state on communication bus.
    ///
    /// If the callback is set, it invokes the callback with the actuator ID.
    virtual void send_state(void);

  protected:
    /// Actuator ID
    Enum id_;

    /// Blocked state: true if actuator is blocked
    bool blocked_;

    /// Callback to send the actuator's current state
    send_state_cb_t send_state_cb_;
};

} // namespace actuators
} // namespace cogip

/// @}
