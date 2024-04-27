// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pami
/// @{
/// @file
/// @brief       C++ class representing a servomotor using lx_servo driver.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "lx_servo.h"

#include "Actuator.hpp"
#include "PB_Actuators.hpp"

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace servos {

enum class Enum: lx_id_t;

std::ostream& operator << (std::ostream& os, Enum id);

/// Class representing a servomotor using lx_servo driver.
class LxServo: public Actuator {
public:
    /// Constructor.
    LxServo(
        Enum id            ///< [in] servo id
    );

    /// Go to the position in a given time.
    /// Return status code.
    lx_comm_error_t move(
        const uint16_t position, ///< [in] the target position
        const uint16_t time = 0  ///< [in] the movement time (default: to full speed)
    );

    /// Register position and moving time.
    /// Return status code.
    lx_comm_error_t move_wait(
        const uint16_t position, ///< [in] the target position
        const uint16_t time = 0  ///< [in] the movement time (default: to full speed)
    );

    /// Go to registered position.
    /// Return status code.
    lx_comm_error_t move_start();

    /// Stop current movement.
    /// Return status code.
    lx_comm_error_t move_stop();

    /// Read current position.
    /// Return the current position or -1 in case of failure.
    int16_t pos_read() const;

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Servo & pb_servo  ///< [out] Protobuf message to fill
    ) const;

    static uart_half_duplex_t *lx_stream;

private:
    Enum id_;
    lx_t lx_;
    uint16_t command_;
};

} // namespace servos
} // namespace actuators
} // namespace pf
} // namespace cogip

/// @}
