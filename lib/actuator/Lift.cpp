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

Lift::Lift(const LiftParameters& params)
    : Motor(params.motor_params,
            // Use DUALPID_TRACKER mode if profile_tracker_parameters is set
            params.motor_params.profile_tracker_parameters != nullptr
                ? MotorControlMode::DUALPID_TRACKER
                : MotorControlMode::DUALPID),
      params_(params)
{
}

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

    int upper_state = gpio_read(params_.upper_limit_switch_pin);
    int lower_state = gpio_read(params_.lower_limit_switch_pin);
    LOG_INFO("Lift init: upper_switch=%d, lower_switch=%d\n", upper_state, lower_state);

    // Note: switches are active-high (read 1 when pressed, 0 when not pressed)
    // This is inverted from typical NO switches with pull-up

    constexpr uint32_t init_timeout_ms = 2000;

    if (!upper_state) {
        LOG_INFO("Moving to upper limit (%d mm)...\n", static_cast<int>(params_.upper_limit_mm));
        actuate_timeout(params_.upper_limit_mm, init_timeout_ms);
        ztimer_sleep(ZTIMER_MSEC, init_timeout_ms);
        LOG_INFO("Upper limit reached or timeout\n");
    } else {
        LOG_INFO("Already at upper limit, setting distance to %d mm\n",
                 static_cast<int>(params_.upper_limit_mm));
        set_current_distance(params_.upper_limit_mm);
    }

    lower_state = gpio_read(params_.lower_limit_switch_pin);
    if (!lower_state) {
        LOG_INFO("Moving to lower limit (%d mm)...\n", static_cast<int>(params_.lower_limit_mm));
        actuate_timeout(params_.lower_limit_mm, init_timeout_ms);
        ztimer_sleep(ZTIMER_MSEC, init_timeout_ms);
        LOG_INFO("Lower limit reached or timeout\n");
    } else {
        LOG_INFO("Already at lower limit, setting distance to %d mm\n",
                 static_cast<int>(params_.lower_limit_mm));
        set_current_distance(params_.lower_limit_mm);
    }

    LOG_INFO("Lift init complete\n");

    // Keep limit switches registered for runtime protection
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

    // Pulse the clear_overload pin to release any motor driver fault
    gpio_clear(params_.motor_params.clear_overload_pin);
    ztimer_sleep(ZTIMER_MSEC, 10);
    gpio_set(params_.motor_params.clear_overload_pin);

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
    // Only react when switch is pressed (reads 1 with active-high logic)
    if (gpio_read(params_.lower_limit_switch_pin)) {
        LOG_INFO("Lower limit switch pressed\n");
        set_current_distance(params_.lower_limit_mm);
        stop();
    } else {
        LOG_INFO("Lower limit switch released\n");
    }
}

void Lift::at_upper_limit()
{
    // Only react when switch is pressed (reads 1 with active-high logic)
    if (gpio_read(params_.upper_limit_switch_pin)) {
        LOG_INFO("Upper limit switch pressed\n");
        set_current_distance(params_.upper_limit_mm);
        stop();
    } else {
        LOG_INFO("Upper limit switch released\n");
    }
}

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip
