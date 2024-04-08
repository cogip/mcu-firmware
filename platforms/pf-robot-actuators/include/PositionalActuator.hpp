// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-actuators
/// @{
/// @file
/// @brief       C++ class representing a positional_actuator using positional_actuator driver.
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include <motor_driver.h>

#include "Actuator.hpp"
#include "PB_Actuators.hpp"

#include <iosfwd>
#include <etl/list.h>

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

typedef int (*check_limit_switch_cb_t)();

enum class Enum: uint8_t;

std::ostream& operator << (std::ostream& os, Enum id);

/// Class representing a positional_actuator using positional_actuator driver.
class PositionalActuator: public Actuator {
public:
    /// Constructor.
    PositionalActuator(
        Enum id,           ///< [in] positional_actuator id
        GroupEnum group,   ///< [in] actuator group
        uint8_t order = 0, ///< [in] order in actuator group
        uint32_t default_timeout_period = 0 ///< [in] default timeout
    ) : Actuator(group, order), id_(id), command_(0), timeout_period_(0), default_timeout_period_(default_timeout_period) {};

    /// Get timeout
    uint32_t timeout_period() { return timeout_period_; };

    /// Decrement timeout
    uint32_t decrement_timeout_period() {
        if (timeout_period_) timeout_period_--;
        return timeout_period_;
    };

    /// Activate the positional_actuator.
    virtual void actuate(
        const int32_t command   ///< [in] positional_actuator speed as a duty_cycle in percent
    ) = 0;

    /// Activate the motor, for the given amount of time
    void actuate_timeout(
        const int32_t command,              ///< [in] motor speed as a duty_cycle in percent
        const uint32_t timeout_period       ///< [in] timeout as a number of thread period
    );

    /// Stop the positional_actuator.
    virtual void disable() = 0;

    /// Disable the positional actuator after conditions.
    virtual bool disable_on_check() = 0;

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_PositionalActuator & pb_positional_actuator  ///< [out] Protobuf message to fill
    ) const;

protected:
    Enum id_;                   ///< positional_actuator id

    int32_t command_;           ///< positional_actuator speed as a duty_cycle in percent

    uint32_t timeout_period_;   ///< timeout to decrease, unit is the actuator thread period

    uint32_t default_timeout_period_;  ///< if not 0, default timeout is applied in case of no timeout set
};

} // namespace positional_actuators
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
