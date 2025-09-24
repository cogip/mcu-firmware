// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       RIOT motor driver implementation
/// @author      COGIP Robotics

#include <cmath>

#include "motor_driver.h"

#include "motor/MotorDriverRIOT.hpp"

namespace cogip {

namespace motor {

int MotorDriverRIOT::init()
{
    return motor_driver_init(&driver_, &parameters_);
}

int MotorDriverRIOT::reset()
{
    /// There is no reset function for RIOT motor driver
    return 0;
}

int MotorDriverRIOT::enable(int id)
{
    motor_enable(&driver_, id);
    return 0;
}

int MotorDriverRIOT::disable(int id)
{
    motor_disable(&driver_, id);
    return 0;
}

int MotorDriverRIOT::set_speed(float speed, int id)
{
    // Convert speed in percent to pwm value
    float pwm_value = (speed * static_cast<float>(parameters_.pwm_resolution)) / 100.0f;

    // limit pwm value in order to ensure range between
    // [-parameters_.pwm_resolution;parameters_.pwm_resolution]
    if (std::fabs(pwm_value) > static_cast<float>(parameters_.pwm_resolution)) {
        pwm_value = (speed < 0.0f ? -1.0f : 1.0f) * static_cast<float>(parameters_.pwm_resolution);
    }

    return motor_set(&driver_, id, static_cast<int32_t>(pwm_value));
}

int MotorDriverRIOT::brake(int id)
{
    return motor_brake(&driver_, id);
}

} // namespace motor

} // namespace cogip
