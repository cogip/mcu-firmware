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
#include "motion_control_common/BaseControllerEngine.hpp"

namespace cogip {

namespace motion_control {

/// Get current speed and pose from motor
using motor_get_speed_and_pose_cb_t = etl::delegate<void(float&, float&)>;

/// Process motion control commands
using motor_process_commands_cb_t = etl::delegate<void(const int, BaseControllerEngine&)>;

/// Engine getting inputs from motor and setting outputs for the motor
class MotorEngine : public BaseControllerEngine {
public:
    /// Constructor
    MotorEngine(
        motor_get_speed_and_pose_cb_t motor_get_speed_and_pose_cb,    ///< [in]  Motor callback to get robot current speed and pose
        motor_process_commands_cb_t motor_process_commands_cb         ///< [in]  Motor callback to process commands output from last controller
    ) : BaseControllerEngine(),
        current_speed_(0), target_speed_(0),
        current_pose_(0), target_pose_(0),
        motor_get_speed_and_pose_cb_(motor_get_speed_and_pose_cb),
        motor_process_commands_cb_(motor_process_commands_cb) {};

    /// Get current speed
    /// return     current speed
    const float& current_speed() const { return current_speed_; };

    /// Get target speed
    /// return     target speed
    const float& target_speed() const { return target_speed_; };

    /// Get current pose
    /// return     current pose
    const float& current_pose() const { return current_pose_; };

    /// Get target pose
    /// return     target pose
    const float& target_pose() const { return target_pose_; };

    /// Set target speed
    void set_target_speed(
        const float target_speed   ///< [in]   new target speed
        ) { target_speed_ = target_speed; };

    /// Set current pose
    void set_current_pose(
        const float current_pose    ///< [in]   new current pose
        ) { current_pose_ = current_pose; };

    /// Set target pose
    void set_target_pose(
        const float target_pose     ///< [in]   new target pose
        ) { target_pose_ = target_pose; };

private:
    /// Prepare controller inputs from motor functions.
    void prepare_inputs();

    /// Process controller output for motor restitution.
    void process_outputs();

    /// Robot polar current speed
    float current_speed_;

    /// Robot polar target speed
    float target_speed_;

    /// Robot current pose
    float current_pose_;

    /// Robot target pose
    float target_pose_;

    /// Motor callback to get target and current poses from motors
    motor_get_speed_and_pose_cb_t motor_get_speed_and_pose_cb_;

    /// Motor calback to drive motors according to commands
    motor_process_commands_cb_t motor_process_commands_cb_;
};

} // namespace motion_control

} // namespace cogip

/// @}
