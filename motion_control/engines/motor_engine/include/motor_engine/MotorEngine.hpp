// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motor_engine Motor engine
/// @{
/// @file
/// @brief      Engine getting inputs from motor and setting outputs for the
/// motor
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// RIOT includes
#include <periph/gpio.h>

// Project includes
#include "etl/delegate.h"
#include "motion_control_common/BaseControllerEngine.hpp"
#include "motor/MotorInterface.hpp"
#include "odometer/OdometerInterface.hpp"

namespace cogip {

namespace motion_control {

/// Engine getting inputs from motor and setting outputs for the motor
class MotorEngine : public BaseControllerEngine
{
  public:
    /// Constructor
    MotorEngine(motor::MotorInterface& motor, localization::OdometerInterface& odometer,
                uint32_t engine_thread_period_ms);

    /// Get target speed
    /// return     target speed
    const float& target_speed() const
    {
        return target_speed_;
    };

    /// Set target speed
    void set_target_speed(const float target_speed ///< [in]   new target speed
    )
    {
        target_speed_ = target_speed;
    };

    /// Get current speed
    /// return     current speed
    float get_current_speed_from_odometer() const
    {
        return odometer_.delta_distance_mm();
    };

    /// Get current distance
    /// return     current distance
    float get_current_distance_from_odometer() const
    {
        return odometer_.distance_mm();
    };

    /// Set current distance
    void set_current_distance_to_odometer(const float distance ///< [in]   new current distance
    );

    /// Get target distance
    /// return     target distance
    const float& target_distance() const
    {
        return target_distance_;
    };

    /// Set target distance
    void set_target_distance(const float target_distance ///< [in]   new target distance
    );

    /// Set GPIO pin used to clear motor driver overload fault.
    /// When set, the pin is driven low at every control period to
    /// continuously reset any overload condition.
    void set_clear_overload_pin(gpio_t pin)
    {
        clear_overload_pin_ = pin;
    }

    /// Set callback invoked once when the target pose is first reached.
    void set_pose_reached_cb(etl::delegate<void(target_pose_status_t)> cb)
    {
        pose_reached_cb_ = cb;
    }

  private:
    /// Prepare controller inputs from motor functions.
    void prepare_inputs();

    /// Process controller output for motor restitution.
    void process_outputs();

    /// Motor polar target speed
    float target_speed_;

    /// Motor target distance
    float target_distance_;

    /// Flag indicating a new target was set (for profile tracker recomputation)
    bool new_target_;

    /// Motor
    motor::MotorInterface& motor_;

    /// EncoderInterface
    localization::OdometerInterface& odometer_;

    /// GPIO pin to clear motor driver overload (GPIO_UNDEF if unused)
    gpio_t clear_overload_pin_ = GPIO_UNDEF;

    /// Stores the pose_reached status from the previous target to notify only once
    target_pose_status_t previous_pose_reached_ = target_pose_status_t::moving;

    /// Callback invoked on pose status transitions (reached, blocked, timeout)
    etl::delegate<void(target_pose_status_t)> pose_reached_cb_;
};

} // namespace motion_control

} // namespace cogip

/// @}
