// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       Base sensor class definition.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {
namespace pf {
namespace sensors {

enum class Enum : uint8_t;

typedef void (*send_state_cb_t)(Enum id);
class Sensor {
public:
    /// Constructor
    explicit Sensor(
        Enum id,                          ///< [in] sensor id
        send_state_cb_t send_state_cb = nullptr   ///< [in] send state callback
    ) : id_(id), state_(false), send_state_cb_(send_state_cb) {};

    /// Get sensor ID
    Enum id() const { return id_; }

    /// Get sensor state
    bool state() const { return state_; }

    /// Set sensor state
    void set_state(bool new_state) { state_ = new_state; }

    /// Send sensor state on communication bus
    virtual void send_state(void) { if (send_state_cb_) send_state_cb_(id_); }

protected:
    Enum id_;                       ///< Sensor ID

    bool state_;                    ///< Current state of the sensor

    send_state_cb_t send_state_cb_; ///< send sensor state on communication bus
};

} // namespace sensors
} // namespace pf
} // namespace cogip

/// @}
