// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       C++ class representing a motor using motor driver.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <motor_driver.h>

#include "PositionalActuator.hpp"

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

enum class Enum: uint8_t;

std::ostream& operator << (std::ostream& os, Enum id);

/// Class representing a motor using motor driver.
class Motor: public PositionalActuator {
public:
    /// Constructor.
    explicit Motor(
        Enum id,                            ///< [in] motor id
        GroupEnum group,                    ///< [in] actuator group
        uint8_t order = 0,                  ///< [in] order in actuator group
        uint32_t default_timeout_period = 0,///< [in] default timeout
        motor_driver_t *motor_driver = nullptr, ///< [in] motor driver
        uint8_t motor_id = 0,               ///< [in] motor id for the given motor driver
        check_limit_switch_cb_t check_limit_switch_positive_direction_cb = nullptr, ///< [in] callback to check limit switch for positive direction
        check_limit_switch_cb_t check_limit_switch_negative_direction_cb = nullptr  ///< [in] callback to check limit switch for negative direction
    ) : PositionalActuator(id, group, order, default_timeout_period),
        motor_driver_(motor_driver),
        motor_id_(motor_id),
        check_limit_switch_positive_direction_cb_(check_limit_switch_positive_direction_cb),
        check_limit_switch_negative_direction_cb_(check_limit_switch_negative_direction_cb) { Motor::disable(); };

    /// Disable the motor after checking direction.
    bool disable_on_check() override;

    /// Disable the motor.
    void disable() override;

    /// Activate the motor.
    void actuate(
        const int32_t command               ///< [in] motor speed as a duty_cycle in percent
    ) override;

private:
    motor_driver_t  *motor_driver_;         ///< hardware motor driver id

    uint8_t motor_id_;                      ///< motor id for the given motor driver

    check_limit_switch_cb_t check_limit_switch_positive_direction_cb_;  ///< callback to check limit switch for positive direction

    check_limit_switch_cb_t check_limit_switch_negative_direction_cb_;  ///< callback to check limit switch for negative direction
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
