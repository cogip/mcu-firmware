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
#include "motion_control_common/ControllersIO.hpp"
#include "PassthroughPosePIDControllerParameters.hpp"

namespace cogip {

namespace motion_control {

/// Pose PID controller.
class PassthroughPosePIDController : public Controller<5, 5, PassthroughPosePIDControllerParameters> {
public:
    /// Constructor
    explicit PassthroughPosePIDController(
        PassthroughPosePIDControllerParameters *parameters     ///< [in]  PID paramaters. See PassthroughPosePIDControllerParameters
        ) : BaseController(), Controller(parameters) {};

    /// Compute PID to correct given error according to PID parameters and inputs.
    /// @param io input/output datas shared accross controllers
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
