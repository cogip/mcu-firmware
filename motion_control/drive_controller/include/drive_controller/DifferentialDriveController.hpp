// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Differential drive controller
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "etl/math_constants.h"
#include "etl/algorithm.h"

#include "cogip_defs/Polar.hpp"

#include "utils.hpp"

#include "DriveControllerInterface.hpp"
#include "DifferentialDriveControllerParameters.hpp"
#include "motor/MotorInterface.hpp"

namespace cogip {

namespace drive_controller {

class DifferentialDriveController: public DriveControllerInterface {
public:
    DifferentialDriveController(DifferentialDriveControllerParameters &parameters,
                                cogip::motor::MotorInterface &left_motor, 
                                cogip::motor::MotorInterface &right_motor) :
                                parameters_(parameters), left_motor_(left_motor), right_motor_(right_motor)
    {
    }

    /// @brief Set polar velocity command
    /// @param command polar velocity command reference
    /// @return  0 on success, negative on error
    int set_polar_velocity(cogip_defs::Polar &command) override
    {
        // Compute wheel speed in mm/period from polar speed
        const double left_wheel_speed_mm_per_period = command.distance() - (command.angle() * parameters_.track_width_mm() / 2);
        const double right_wheel_speed_mm_per_period = command.distance() + (command.angle() * parameters_.track_width_mm() / 2);

        // Compute wheel speed in mm/s and rad/s from mm/period and rad/period speeds
        const double left_wheel_speed_mm_per_s = left_wheel_speed_mm_per_period * 1000 / parameters_.loop_period_ms();
        const double right_wheel_speed_mm_per_s = right_wheel_speed_mm_per_period * 1000/ parameters_.loop_period_ms();

        // Compute motor speed in percent using the motor constant.
        // The motor constant allow convert a speed in mm/s into a speed ratio (% of nominal motor voltage).
        const double left_motor_speed_percent = left_wheel_speed_mm_per_s / (etl::math::pi * parameters_.left_wheel_diameter_mm()) * parameters_.left_motor_constant();
        const double right_motor_speed_percent = right_wheel_speed_mm_per_s / (etl::math::pi * parameters_.right_wheel_diameter_mm()) * parameters_.right_motor_constant();

        // Apply motor speed
        left_motor_.speed((int)etl::clamp(left_motor_speed_percent, -parameters_.max_speed_percentage(), parameters_.max_speed_percentage()));
        right_motor_.speed((int)etl::clamp(right_motor_speed_percent, -parameters_.max_speed_percentage(), parameters_.max_speed_percentage()));

        return 0;
    }

private:
    DifferentialDriveControllerParameters &parameters_;

    cogip::motor::MotorInterface &left_motor_;
    cogip::motor::MotorInterface &right_motor_;
};

} // namespace drive_controller

} // namespace cogip

// @}
