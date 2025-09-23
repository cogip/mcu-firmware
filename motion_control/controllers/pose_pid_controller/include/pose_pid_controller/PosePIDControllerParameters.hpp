// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_pid_controller Pose PID controller parameters
/// @{
/// @file
/// @brief      Pose PID controller parameters
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "pid/PID.hpp"

namespace cogip {

namespace motion_control {

/// Pose controller parameters
class PosePIDControllerParameters {
public:
    /// Constructor
    explicit PosePIDControllerParameters(
        pid::PID *pid = nullptr     ///< [in]  PID parameters
    ) : pid_(pid) {};

    /// Get PID parameters
    /// return     PID parameters pointer
    pid::PID *pid() const { return pid_; };

private:
    /// PID parameters
    pid::PID *pid_;   ///< Position PID
};

} // namespace motion_control

} // namespace cogip

/// @}
