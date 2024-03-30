// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pami
/// @{
/// @file
/// @brief       C++ class representing a on/off actuator using GPIOs.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "PositionalActuator.hpp"

#include <iosfwd>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

/// Class representing a motor using motor driver.
class OnOff: public PositionalActuator {
public:
    /// Constructor.
    OnOff(
        Enum id,                            ///< [in] motor id
        GroupEnum group,                    ///< [in] actuator group
        uint8_t order = 0,                  ///< [in] order in actuator group
        uint32_t default_timeout_period = 0,///< [in] default timeout
        bool use_gpio_expander = false,     ///< [in] false if native GPIO, true for expander
        bool active_state = true,           ///< [in] Consider On when GPIO is equal to active_state
        gpio_t pin = GPIO_UNDEF             ///< [in] On/Off GPIO
    ) : PositionalActuator(id, group, order, default_timeout_period), use_gpio_expander_(use_gpio_expander), active_state_(active_state), pin_(pin) {};

    /// Disable the motor.
    void disable() override;

    /// Disable the positional actuator after conditions.
    bool disable_on_check() override { disable(); return true; };

    /// Activate the actuator.
    void actuate(
        const int32_t command               ///< [in] motor speed as a duty_cycle in percent
    ) override;

private:
    bool    use_gpio_expander_;              ///< [in] false if native GPIO, true for expander
    bool    active_state_;                   ///< [in] Consider On when GPIO is equal to active_state
    gpio_t  pin_;                            ///< [in] On/Off GPIO
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
