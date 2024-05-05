// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       C++ class representing a on/off actuator using GPIOs.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "PositionalActuator.hpp"

#include <etl/vector.h>
#include <pca9685.h>

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

/// Class representing a servomotor driven by pca9685.
class AnalogServo: public PositionalActuator {
public:
    /// Constructor.
    AnalogServo(
        Enum id,                            ///< [in] motor id
        uint32_t default_timeout_period = 0,///< [in] default timeout
        send_state_cb_t send_state_cb = nullptr, ///< [in] send state callback
        int servo_id = 0                    ///< [in] Servomotor ID on PCA9685
    );

    /// Disable the motor.
    void disable() override;

    /// Disable the positional actuator after conditions.
    bool disable_on_check() override { disable(); return true; };

    /// Activate the motor.
    void actuate(
        const int32_t command               ///< [in] servomotor position
    ) override;

    static pca9685_t pca9685_dev;           ///< PCA9685 I2C PWM driver

private:
    int channel_ = 0;                       ///< Servomotor ID on PCA9685
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
