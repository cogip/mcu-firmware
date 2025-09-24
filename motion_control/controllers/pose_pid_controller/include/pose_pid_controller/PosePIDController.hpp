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
#include "PosePIDControllerIOKeys.hpp"
#include "PosePIDControllerParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {

namespace motion_control {

/// @brief Pose PID controller.
///        Reads “position_error” and computes “speed_order” via PID.
class PosePIDController : public Controller<PosePIDControllerIOKeys, PosePIDControllerParameters>
{
  public:
    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys.
    /// @param parameters Reference to PID parameters.
    explicit PosePIDController(const PosePIDControllerIOKeys& keys,
                               const PosePIDControllerParameters& parameters)
        : Controller<PosePIDControllerIOKeys, PosePIDControllerParameters>(keys, parameters)
    {
    }

    /// @brief Read position error via keys_.position_error,
    ///        compute speed order, and write it to controller IO map.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
