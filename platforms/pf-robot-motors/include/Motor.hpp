// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     platforms_pf-robot-motors
/// @{
/// @file
/// @brief       C++ class representing a motor using motor driver.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <motor_driver.h>

#include <iosfwd>

#include "PositionalActuator.hpp"

#include "motor_engine/MotorEngine.hpp"
#include "motor_pose_filter/MotorPoseFilter.hpp"
#include "motor_pose_filter/MotorPoseFilterParameters.hpp"
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

namespace cogip {
namespace pf {
namespace actuators {
namespace positional_actuators {

enum class Enum: uint8_t;

std::ostream& operator << (std::ostream& os, Enum id);

/// Class representing a motor using motor driver.
class Motor: public PositionalActuator {
public:
    /// Constructor.
    explicit Motor(
        cogip::pf::actuators::Enum id,          ///< [in] motor id
        uint32_t default_timeout_period = 0,    ///< [in] default timeout
        send_state_cb_t send_state_cb = nullptr,///< [in] send state callback
        motor_driver_t *motor_driver = nullptr, ///< [in] motor driver
        uint8_t motor_id = 0,                   ///< [in] motor id for the given motor driver
        gpio_t clear_overload_pin = GPIO_UNDEF, ///< [in] clear motor overload flag
        double target_speed = 0,                ///< [in] motor engine default target speed
        cogip::motion_control::PosePIDControllerParameters *pose_controller_parameters = nullptr,
                                                ///< [in] motor pose controller parameters
        cogip::motion_control::SpeedPIDControllerParameters *speed_controller_parameters = nullptr,
                                                ///< [in] motor speed controller parameters
        cogip::motion_control::MotorPoseFilterParameters *motor_pose_filter_parameters = nullptr,
                                                ///< [in] motor pose filter parameters
        cogip::motion_control::SpeedFilterParameters *speed_filter_parameters = nullptr,
                                                ///< [in] motor speed filter parameters
        cogip::motion_control::motor_get_speed_and_pose_cb_t motor_get_speed_and_pose_cb =
            cogip::motion_control::motor_get_speed_and_pose_cb_t::create<nullptr>(),
                                                ///< [in]  motor callback to get robot current speed and pose
        cogip::motion_control::motor_process_commands_cb_t motor_process_commands_cb =
            cogip::motion_control::motor_process_commands_cb_t::create<nullptr>()
                                                ///< [in]  motor callback to process commands output from last controller
    );

    /// Disable the motor.
    void disable() override;

    /// Disable the positional actuator after conditions.
    bool disable_on_check() override { disable(); return true; };

    /// Activate the motor.
    void actuate(
        const int32_t command               ///< [in] motor speed as a duty_cycle in percent
    ) override;

    /// Get target speed
    /// return target speed
    double target_speed() const { return motor_engine_.target_speed(); };

    /// Set target speed
    void set_target_speed(
        double target_speed         ///< [in]   target speed (mm/period)
        ) { motor_engine_.set_target_speed(target_speed); };

    /// Get current pose
    /// return current pose
    double current_pose() const { return motor_engine_.current_pose(); };

    /// Set current pose
    void set_current_pose(
        double current_pose     ///< [in]   current pose (mm)
        ) { motor_engine_.set_current_pose(current_pose); };

private:
    /// Hardware motor driver
    motor_driver_t  *motor_driver_;

    /// Motor id for the given motor driver
    uint8_t motor_id_;

    /// Motor overload clearance gpio
    gpio_t clear_overload_pin_;

    /// DualPIDMetaController for pose and speed control in cascade.
    cogip::motion_control::DualPIDMetaController dualpid_meta_controller_;

    /// PosePIDController that provides SpeedPIDController order first filtered by SpeedFilter.
    cogip::motion_control::PosePIDController pose_controller_;

    /// SpeedPIDController to compute command to send to motors.
    cogip::motion_control::SpeedPIDController speed_controller_;

    /// MotorPoseFilter to check if pose reached.
    cogip::motion_control::MotorPoseFilter motor_pose_filter_;

    /// SpeedFilter to limit speed and acceleration for SpeedPIDController.
    cogip::motion_control::SpeedFilter speed_filter_;

    /// Motor controllers engine
    cogip::motion_control::MotorEngine motor_engine_;
};

} // namespace motors
} // namespace actuators
} // namespace app
} // namespace cogip

/// @}
