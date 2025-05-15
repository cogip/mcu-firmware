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
}

void Lift::init() {
    int ret = LiftsLimitSwitchesManager::instance().register_gpio(
        params_.lower_limit_switch_pin,
        this
    );
    if (ret) {
        std::cerr << "Lower switch pin for lift " << id_ << " registering failed" << std::endl;
    }
    ret = LiftsLimitSwitchesManager::instance().register_gpio(
        params_.upper_limit_switch_pin,
        this
    );
    if (ret) {
        std::cerr << "Upper switch pin for lift " << id_ << " registering failed" << std::endl;
    }

    // Homing sequence: move to lower, then upper end-stop at lower speed
    set_target_speed_percent(params_.init_speed_percentage);

    if (gpio_read(params_.upper_limit_switch_pin)) {
        Motor::actuate(params_.upper_limit_mm);
        ztimer_sleep(ZTIMER_MSEC, motor_engine_.timeout_ms());
    }
    else {
        set_current_distance(params_.upper_limit_mm);
    }
    if (gpio_read(params_.lower_limit_switch_pin)) {
        Motor::actuate(params_.lower_limit_mm);
        ztimer_sleep(ZTIMER_MSEC, motor_engine_.timeout_ms());
    }
    else {
        set_current_distance(params_.lower_limit_mm);
    }

    ret = LiftsLimitSwitchesManager::instance().unregister_gpio(
        params_.lower_limit_switch_pin
    );
    if (ret) {
        std::cerr << "Lower switch pin for lift " << id_ << " unregistering failed" << std::endl;
    }
    ret = LiftsLimitSwitchesManager::instance().unregister_gpio(
        params_.upper_limit_switch_pin
    );
    if (ret) {
        std::cerr << "Upper switch pin for lift " << id_ << " unregistering failed" << std::endl;
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
    const int32_t clamped = (command < params_.lower_limit_mm) ? params_.lower_limit_mm
                          : (command > params_.upper_limit_mm) ? params_.upper_limit_mm
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
    set_current_distance(params_.lower_limit_mm);
}

void Lift::at_upper_limit()
{
    std::cout << "Upper limit switch triggered" << std::endl;
    set_current_distance(params_.upper_limit_mm);
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
