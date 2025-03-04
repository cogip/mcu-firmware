// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_pid_controller Pose PID controller
/// @{
/// @file
/// @brief      Pose PID controller
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/Controller.hpp"
#include "PassthroughPosePIDControllerParameters.hpp"

namespace cogip {

namespace motion_control {

/// Pose PID controller.
/// Input 0:    polar pose error
/// Input 1:    current speed
/// Input 2:    target speed
/// Input 3:    filter speed bool
/// Input 4:    pose reached
/// Output 0:   speed order
/// Output 1:   current speed
/// Output 2:   target speed
/// Output 3:   filter speed bool
/// Output 4:   pose reached
class PassthroughPosePIDController : public Controller<5, 5, PassthroughPosePIDControllerParameters> {
public:
    /// Constructor
    explicit PassthroughPosePIDController(
        PassthroughPosePIDControllerParameters *parameters     ///< [in]  PID paramaters. See PassthroughPosePIDControllerParameters
        ) : BaseController(), Controller(parameters) {};

    /// Compute PID to correct given error according to PID parameters and inputs.
    void execute() override;
};

} // namespace motion_control

} // namespace cogip

/// @}
