// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       C++ class representing a motor using motor driver.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include <motor_driver.h>

#include "Actuator.hpp"
#include "PB_Actuators.hpp"

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace motors {

enum class Enum: uint8_t;

std::ostream& operator << (std::ostream& os, Enum id);

/// Class representing a motor using motor driver.
class Motor: public Actuator {
public:
    /// Constructor.
    Motor(
        Enum id,           ///< [in] motor id
        GroupEnum group,   ///< [in] actuator group
        uint8_t order = 0  ///< [in] order in actuator group
    );

    /// Activate the motor.
    void move(
        bool direction,         ///< [in] motor rotation direction
        const uint32_t speed    ///< [in] motor speed as a duty_cycle in percent
    );

    /// Stop the motor.
    void deactivate();

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Motor & pb_motor  ///< [out] Protobuf message to fill
    ) const;

private:
    Enum id_;           ///< motor id
    bool activated_;    ///< whether motor is activated or not
    bool direction_;    ///< motor rotation direction
    uint32_t speed_;     ///< motor speed as a duty_cycle in percent
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
