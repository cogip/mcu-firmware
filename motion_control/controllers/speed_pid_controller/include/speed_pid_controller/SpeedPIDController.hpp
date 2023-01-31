// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_pid_controller Speed PID controller
/// @{
/// @file
/// @brief      Speed PID controller
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/Controller.hpp"
#include "SpeedPIDControllerParameters.hpp"

namespace cogip {

namespace motion_control {

/// PID speed controller.
/// Input 0:    speed order
/// Input 1:    current speed
/// Input 2:    target speed
/// Output 0:   speed command
class SpeedPIDController : public Controller<1, 1, SpeedPIDControllerParameters> {
public:
    /// Constructor
    explicit SpeedPIDController(
        SpeedPIDControllerParameters *parameters    ///< [in]  PID parameters. See SpeedPIDControllerParameters.
        ) : BaseController(), Controller(parameters) {};

    /// Compute PID to correct given error according to PID parameters and inputs.
    void execute() override;
};

} // namespace motion_control

} // namespace cogip

/// @}
