// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include <periph/gpio.h>

#include "actuator/Lift.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

Lift::Lift(const LiftParameters& params)
    : Motor(params.motor_params),
    params_(params)
{

    if (LiftsLimitSwitchesManager::instance().register_gpio(
        params_.lower_limit_switch_pin,
        this
    )) {
        std::cerr << "Lower switch pin for lift " << id_ << " failed" << std::endl;
    }
    if (LiftsLimitSwitchesManager::instance().register_gpio(
        params_.upper_limit_switch_pin,
        this
    )) {
        std::cerr << "Lower switch pin for lift " << id_ << " failed" << std::endl;
    }
}

void Lift::init() {
    // Homing sequence: move to lower, then upper end-stop at lower speed
    set_target_speed_percent(params_.init_speed_percentage);

    if (gpio_read(params_.upper_limit_switch_pin)) {
        Motor::actuate(INT16_MAX);
        ztimer_sleep(ZTIMER_MSEC, motor_engine_.timeout_ms());
    }
    else {
        set_current_distance(params_.upper_limit_ms);
    }
    if (gpio_read(params_.lower_limit_switch_pin)) {
        Motor::actuate(INT16_MIN);
        ztimer_sleep(ZTIMER_MSEC, motor_engine_.timeout_ms());
    }
    else {
        set_current_distance(params_.lower_limit_ms);
    }
}

void Lift::stop()
{
    actuate(get_current_distance());
}

void Lift::actuate(const int32_t command)
{
    std::cout << "Move lift to command " << command << std::endl;
    // clamp within mechanical bounds
    const int32_t clamped = (command < params_.lower_limit_ms) ? params_.lower_limit_ms
                          : (command > params_.upper_limit_ms) ? params_.upper_limit_ms
                          : command;

    std::cout << "Move lift to clamped command " << clamped << std::endl;

    Motor::actuate(clamped);
}

void Lift::at_limits(gpio_t pin)
{
    if (pin == params_.lower_limit_switch_pin) {
        at_lower_limit();
        return;
    }
    else if (pin == params_.upper_limit_switch_pin) {
        at_upper_limit();
        return;
    }
    else {
        std::cerr << "Pin not handled by this lift" << std::endl;
    }
}

void Lift::at_lower_limit()
{
    std::cout << "Lower limit switch triggered" << std::endl;
    set_current_distance(params_.lower_limit_ms);
    actuate(params_.lower_limit_ms);
}

void Lift::at_upper_limit()
{
    std::cout << "Upper limit switch triggered" << std::endl;
    set_current_distance(params_.upper_limit_ms);
    actuate(params_.upper_limit_ms);
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
