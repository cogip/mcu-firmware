// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "log.h"
#include <inttypes.h>
#include <periph/gpio.h>

#include "actuator/Lift.hpp"
#include "actuator/LiftsLimitSwitchesManager.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

Lift::Lift(const LiftParameters& params) : Motor(params.motor_params), params_(params) {}

void Lift::init()
{
    int ret =
        LiftsLimitSwitchesManager::instance().register_gpio(params_.lower_limit_switch_pin, this);
    if (ret) {
        LOG_ERROR("Lower switch pin for lift %" PRIu8 " registering failed\n",
                  static_cast<uint8_t>(id_));
    }
    ret = LiftsLimitSwitchesManager::instance().register_gpio(params_.upper_limit_switch_pin, this);
    if (ret) {
        LOG_ERROR("Upper switch pin for lift %" PRIu8 " registering failed\n",
                  static_cast<uint8_t>(id_));
    }

    // Homing sequence: move to lower, then upper end-stop at lower speed
    set_target_speed_percent(params_.init_speed_percentage);

    if (gpio_read(params_.upper_limit_switch_pin)) {
        actuate(params_.upper_limit_mm);
        ztimer_sleep(ZTIMER_MSEC, motor_engine_.timeout_ms());
    } else {
        set_current_distance(params_.upper_limit_mm);
    }
    if (gpio_read(params_.lower_limit_switch_pin)) {
        actuate(params_.lower_limit_mm);
        ztimer_sleep(ZTIMER_MSEC, motor_engine_.timeout_ms());
    } else {
        set_current_distance(params_.lower_limit_mm);
    }

    ret = LiftsLimitSwitchesManager::instance().unregister_gpio(params_.lower_limit_switch_pin);
    if (ret) {
        LOG_ERROR("Lower switch pin for lift %" PRIu8 " unregistering failed\n",
                  static_cast<uint8_t>(id_));
    }
    ret = LiftsLimitSwitchesManager::instance().unregister_gpio(params_.upper_limit_switch_pin);
    if (ret) {
        LOG_ERROR("Upper switch pin for lift %" PRIu8 " unregistering failed\n",
                  static_cast<uint8_t>(id_));
    }
}

void Lift::stop()
{
    actuate(get_current_distance());
}

void Lift::actuate(int32_t command)
{
    LOG_INFO("Move lift to command %" PRIi32 "\n", command);
    // clamp within mechanical bounds
    const int32_t clamped = (command < params_.lower_limit_mm)   ? params_.lower_limit_mm
                            : (command > params_.upper_limit_mm) ? params_.upper_limit_mm
                                                                 : command;

    LOG_INFO("Move lift to clamped command %" PRIi32 "\n", clamped);

    if (clamped >= motor_engine_.get_current_distance_from_odometer()) {
        gpio_clear(params_.motor_params.clear_overload_pin);
    } else {
        gpio_clear(params_.motor_params.clear_overload_pin);
        ztimer_sleep(ZTIMER_MSEC, 10);
        gpio_set(params_.motor_params.clear_overload_pin);
    }

    Motor::actuate(clamped);
}

void Lift::at_limits(gpio_t pin)
{
    if (pin == params_.lower_limit_switch_pin) {
        at_lower_limit();
        return;
    } else if (pin == params_.upper_limit_switch_pin) {
        at_upper_limit();
        return;
    } else {
        LOG_ERROR("Pin not handled by this lift\n");
    }
}

void Lift::at_lower_limit()
{
    LOG_INFO("Lower limit switch triggered\n");
    set_current_distance(params_.lower_limit_mm);
}

void Lift::at_upper_limit()
{
    LOG_INFO("Upper limit switch triggered\n");
    set_current_distance(params_.upper_limit_mm);
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
