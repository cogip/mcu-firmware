// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Differential drive controller implementation
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#include "etl/math_constants.h"
#include "etl/algorithm.h"
#include "etl/absolute.h"

#include "cogip_defs/Polar.hpp"
#include "utils.hpp"
#include "trigonometry.h"

#include "drive_controller/DifferentialDriveController.hpp"

namespace cogip {

namespace drive_controller {

DifferentialDriveController::DifferentialDriveController(DifferentialDriveControllerParameters &parameters,
                            motor::MotorInterface &left_motor,
                            motor::MotorInterface &right_motor):
                            parameters_(parameters), left_motor_(left_motor), right_motor_(right_motor)
{
}

int DifferentialDriveController::set_polar_velocity(cogip_defs::Polar &command)
{
    // Compute wheel speed in mm/period from polar speed
    const float left_wheel_speed_mm_per_period = command.distance() - (DEG2RAD(command.angle()) * parameters_.track_width_mm() / 2);
    const float right_wheel_speed_mm_per_period = command.distance() + (DEG2RAD(command.angle()) * parameters_.track_width_mm() / 2);

    // Compute wheel speed in mm/s and rad/s from mm/period and rad/period speeds
    const float left_wheel_speed_mm_per_s = left_wheel_speed_mm_per_period * 1000 / parameters_.loop_period_ms();
    const float right_wheel_speed_mm_per_s = right_wheel_speed_mm_per_period * 1000 / parameters_.loop_period_ms();

    // Compute motor speed in percent using the motor constant.
    // The motor constant allow convert a speed in mm/s into a speed ratio (% of nominal motor voltage).
    float left_motor_speed_percent = (left_wheel_speed_mm_per_s / (etl::math::pi * parameters_.left_wheel_diameter_mm())) * parameters_.left_motor_constant();
    float right_motor_speed_percent = (right_wheel_speed_mm_per_s / (etl::math::pi * parameters_.right_wheel_diameter_mm())) * parameters_.right_motor_constant();

    if (etl::absolute(left_motor_speed_percent) < parameters_.min_speed_percentage()) {
        left_motor_speed_percent = (left_motor_speed_percent < 0 ? -1 : 1) * parameters_.min_speed_percentage();
    }
    if (etl::absolute(right_motor_speed_percent) < parameters_.min_speed_percentage()) {
        right_motor_speed_percent = (right_motor_speed_percent < 0 ? -1 : 1) * parameters_.min_speed_percentage();
    }


    // Apply motor speed
    left_motor_.set_speed(etl::clamp(left_motor_speed_percent, -parameters_.max_speed_percentage(), parameters_.max_speed_percentage()));
    right_motor_.set_speed(etl::clamp(right_motor_speed_percent, -parameters_.max_speed_percentage(), parameters_.max_speed_percentage()));

    return 0;
}

} // namespace drive_controller

} // namespace cogip