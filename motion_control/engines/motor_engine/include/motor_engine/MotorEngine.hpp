// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motor_engine Motor engine
/// @{
/// @file
/// @brief      Engine getting inputs from motor and setting outputs for the motor
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "etl/delegate.h"
#include "odometer/OdometerInterface.hpp"
#include "motion_control_common/BaseControllerEngine.hpp"
#include "motor/MotorInterface.hpp"

namespace cogip {

namespace motion_control {

/// Engine getting inputs from motor and setting outputs for the motor
class MotorEngine : public BaseControllerEngine {
public:
    /// Constructor
    MotorEngine(
        cogip::motor::MotorInterface& motor,
        cogip::localization::OdometerInterface& odometer,
        uint32_t engine_thread_period_ms
    ) : BaseControllerEngine(engine_thread_period_ms),
        target_speed_(0),
        target_distance_(0),
        motor_(motor),
        odometer_(odometer) {};

    /// Get target speed
    /// return     target speed
    const float& target_speed() const { return target_speed_; };

    /// Set target speed
    void set_target_speed(
        const float target_speed   ///< [in]   new target speed
        ) { target_speed_ = target_speed; };

    /// Get current speed
    /// return     current speed
    float get_current_speed_from_odometer() const { return odometer_.delta_distance_mm(); };

    /// Get current distance
    /// return     current distance
    float get_current_distance_from_odometer() const { return odometer_.distance_mm(); };

    /// Set current distance
    void set_current_distance_to_odometer(
        const float distance    ///< [in]   new current distance
        ) {
        mutex_lock(&mutex_);
        odometer_.set_distance_mm(distance);
        mutex_unlock(&mutex_);
    };

    /// Get target distance
    /// return     target distance
    const float& target_distance() const { return target_distance_; };

    /// Set target distance
    void set_target_distance(
        const float target_distance     ///< [in]   new target distance
    ) {
        mutex_lock(&mutex_);
        target_distance_ = target_distance;

        // New target, reset the timeout
        timeout_cycle_counter_ = timeout_ms_ / engine_thread_period_ms_;

        // Reset the pose reached flag
        pose_reached_ = target_pose_status_t::moving;

        mutex_unlock(&mutex_);
    };

private:
    /// Prepare controller inputs from motor functions.
    void prepare_inputs();

    /// Process controller output for motor restitution.
    void process_outputs();

    /// Motor polar target speed
    float target_speed_;

    /// Motor target distance
    float target_distance_;

    /// Motor
    cogip::motor::MotorInterface& motor_;

    /// EncoderInterface
    cogip::localization::OdometerInterface& odometer_;
};

} // namespace motion_control

} // namespace cogip

/// @}
