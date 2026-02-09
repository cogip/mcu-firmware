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
};

} // namespace motion_control

} // namespace cogip

/// @}
