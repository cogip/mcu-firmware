// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     actuator
/// @{
/// @file
/// @brief       C++ class representing a motor using a motor driver and cascaded PID+filter control.
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "actuator/PositionalActuator.hpp"
#include "actuator/MotorParameters.hpp"
// Motion control
#include "dualpid_meta_controller/DualPIDMetaController.hpp"
#include "motor_engine/MotorEngine.hpp"
#include "motor_pose_filter/MotorPoseFilter.hpp"
#include "motor_pose_filter/MotorPoseFilterParameters.hpp"
#include "pose_pid_controller/PosePIDController.hpp"
#include "pose_pid_controller/PosePIDControllerParameters.hpp"
#include "speed_filter/SpeedFilter.hpp"
#include "speed_filter/SpeedFilterParameters.hpp"
#include "speed_pid_controller/SpeedPIDController.hpp"
#include "speed_pid_controller/SpeedPIDControllerParameters.hpp"

namespace cogip {
namespace actuators {
namespace positional_actuators {

/// @brief Stream insertion operator for actuator Enum ID.
/// @param os   Output stream.
/// @param id   Actuator Enum ID to insert.
/// @return     Reference to the output stream.
std::ostream& operator<<(std::ostream& os, Enum id);

/// @brief Class representing a motor using a motor driver and cascaded PID+filter control.
/// @details
///   Instantiates and configures all sub-controllers and the control engine according
///   to the parameters passed in @ref MotorParameters.
class Motor : public PositionalActuator {
public:
    /// @brief Construct a Motor from its static parameter set.
    /// @param motor_parameters Reference to a `MotorParameters` in static storage.
    explicit Motor(const MotorParameters& motor_parameters);

    /// @brief Perform the initialization sequence (e.g., homing).
    void init();

    /// @brief Disable the motor immediately (brake + stop control engine).
    void disable() override;

    /// @brief Enable the motor.
    void enable() override;

    /// @brief Request a new target distance for the motor.
    /// @param command Desired target distance (in encoder units or mm).
    void actuate(int32_t command) override;

    /// @brief Return the target speed as a percentage of the maximum speed.
    /// @return Target speed (in %) relative to max speed.
    float get_target_speed_percentage() const override {
        return params_.speed_filter_parameters.max_speed() == 0.0f
            ? 0.0f
            : (motor_engine_.target_speed() / params_.speed_filter_parameters.max_speed()) * 100.0f;
    }

    /// @brief Set the target speed as a percentage of the maximum speed.
    /// @param percentage Target speed (in %) relative to max speed.
    void set_target_speed_percent(float percentage) override {
        motor_engine_.set_target_speed(
            (percentage / 100.0f) * params_.speed_filter_parameters.max_speed()
        );
    }

    /// @brief Get the current measured distance.
    /// @return Current distance in mm.
    float get_current_distance() const { return motor_engine_.get_current_distance_from_odometer(); }

    /// @brief Manually override the current distance reading.
    /// @param distance New distance in mm.
    void set_current_distance(float distance) { motor_engine_.set_current_distance_to_odometer(distance); }

protected:
    /// Reference to the static parameter set.
    const MotorParameters& params_;

    /// Cascade controller combining pose and speed control.
    cogip::motion_control::DualPIDMetaController    dualpid_meta_controller_;

    /// Position (pose) PID controller.
    cogip::motion_control::PosePIDController        distance_controller_;

    /// Speed PID controller.
    cogip::motion_control::SpeedPIDController       speed_controller_;

    /// Filter for pose (distance) measurement.
    cogip::motion_control::MotorPoseFilter          motor_distance_filter_;

    /// Filter for speed/acceleration measurement.
    cogip::motion_control::SpeedFilter              speed_filter_;

    /// Threaded control engine managing the control loop.
    cogip::motion_control::MotorEngine              motor_engine_;
};

} // namespace positional_actuators
} // namespace actuators
} // namespace cogip

/// @}
