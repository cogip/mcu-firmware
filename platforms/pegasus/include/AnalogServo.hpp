// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pegasus
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

constexpr uint8_t max_positions = 10;

/// Class representing a servomotor driven by pca9685.
class AnalogServo: public PositionalActuator {
public:
    /// Constructor.
    AnalogServo(
        Enum id,                            ///< [in] motor id
        GroupEnum group,                    ///< [in] actuator group
        uint8_t order = 0,                  ///< [in] order in actuator group
        uint32_t default_timeout_period = 0,///< [in] default timeout
        int servo_id = 0                    ///< [in] Servomotor ID on PCA9685
    );

    /// Add new position
    void add_position(uint16_t);

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

    etl::vector<uint16_t, max_positions> positions_;    ///< List of servomotor positions
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
