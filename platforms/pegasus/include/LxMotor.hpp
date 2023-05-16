// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
/// @{
/// @file
/// @brief       C++ class representing a servomotor in continuous mode using lx_servo driver.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "PositionalActuator.hpp"
#include "lx_servo.h"

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

/// Class representing a LX Servomotor in motor mode.
class LxMotor: public PositionalActuator {
public:
    /// Constructor.
    LxMotor(
        Enum id,                            ///< [in] motor id
        GroupEnum group,                    ///< [in] actuator group
        uint8_t order = 0,                  ///< [in] order in actuator group
        uint32_t default_timeout_period = 0,///< [in] default timeout
        lx_id_t lx_id = 1,                  ///< [in] LX Servomotor ID
        check_limit_switch_cb_t check_limit_switch_positive_direction_cb = nullptr, ///< [in] callback to check limit switch for positive direction
        check_limit_switch_cb_t check_limit_switch_negative_direction_cb = nullptr  ///< [in] callback to check limit switch for negative direction
    );

    /// Disable the motor.
    void disable();

    /// Disable the positional actuator after conditions.
    bool disable_on_check();

    /// Activate the motor.
    void actuate(
        const int32_t command               ///< [in] motor speed as a duty_cycle in percent
    );

    static uart_half_duplex_t *lx_stream;   ///< Serial half-duplex stream to communicate with LX servomotors

private:
    lx_t lx_;                               ///< LX Servomotor device

    lx_id_t lx_id_;                         ///< LX Servomotor ID

    check_limit_switch_cb_t check_limit_switch_positive_direction_cb_;  ///< callback to check limit switch for positive direction

    check_limit_switch_cb_t check_limit_switch_negative_direction_cb_;  ///< callback to check limit switch for negative direction
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
