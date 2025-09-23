// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       Differential drive controller
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "DifferentialDriveControllerParameters.hpp"
#include "DriveControllerInterface.hpp"
#include "cogip_defs/Polar.hpp"
#include "motor/MotorInterface.hpp"

namespace cogip {

namespace drive_controller {

class DifferentialDriveController : public DriveControllerInterface
{
  public:
    DifferentialDriveController(DifferentialDriveControllerParameters& parameters,
                                motor::MotorInterface& left_motor,
                                motor::MotorInterface& right_motor);

    /// @brief Set polar velocity command
    /// @param command polar velocity command reference
    /// @return  0 on success, negative on error
    int set_polar_velocity(cogip_defs::Polar& command) override;

  private:
    DifferentialDriveControllerParameters& parameters_;

    motor::MotorInterface& left_motor_;
    motor::MotorInterface& right_motor_;
};

} // namespace drive_controller

} // namespace cogip

// @}
