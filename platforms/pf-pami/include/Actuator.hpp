// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pami
/// @{
/// @file
/// @brief       Base actuators class definition.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

namespace cogip {
namespace pf {
namespace actuators {

enum class GroupEnum: uint8_t;
enum class Enum: uint8_t;

typedef void (*send_state_cb_t)(Enum id);
class Actuator {
public:
    /// Constructor
    explicit Actuator(
        Enum id,                                ///< [in] actuator id
        send_state_cb_t send_state_cb = nullptr ///< [in] send state callback
    ) : id_(id), blocked_(0), send_state_cb_(send_state_cb) {};

    /// Get blocked state
    bool blocked() { return blocked_; }

    /// Set blocked state
    void set_blocked(bool blocked) { blocked_ = blocked; }

    /// Send actuator state on communication bus
    virtual void send_state(void) { if (send_state_cb_) send_state_cb_(id_); }

protected:
    Enum id_;           /// Actuator ID

    bool blocked_;      /// blocked state

    send_state_cb_t send_state_cb_; ///< send actuator current state on communication bus
};

} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
