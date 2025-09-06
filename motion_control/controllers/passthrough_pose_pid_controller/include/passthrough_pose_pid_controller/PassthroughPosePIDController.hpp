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
#include "PassthroughPosePIDControllerIOKeys.hpp"

namespace cogip {

namespace motion_control {

/// @brief Pose PID controller: reads “position_error”, computes a “speed_order”,
///        and writes “speed_order” + “target_speed” into ControllersIO.
///        Both the key names and the PID parameters are supplied by the caller.
class PassthroughPosePIDController : public Controller<
    PassthroughPosePIDControllerIOKeys,
    PassthroughPosePIDControllerParameters> {
public:
    /// @brief Constructor.
    /// @param keys        Pointer to a POD containing the three key names.
    /// @param parameters  Reference to PID parameters.
    explicit PassthroughPosePIDController(
        const PassthroughPosePIDControllerIOKeys&       keys,
        const PassthroughPosePIDControllerParameters&   parameters
    )
        : Controller<PassthroughPosePIDControllerIOKeys,
                     PassthroughPosePIDControllerParameters>(keys, parameters) {}

    /// @brief Read “position_error” via keys_->position_error, compute speed order,
    ///        and write “speed_order” + “target_speed” back via keys_->speed_order/target_speed.
    /// @param io Shared ControllersIO.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
