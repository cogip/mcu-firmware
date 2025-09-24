// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_pid_controller Speed PID controller parameters
/// @{
/// @file
/// @brief      Speed PID controller parameters
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "pid/PID.hpp"

namespace cogip {

namespace motion_control {

/// Speed PID controller parameters
class SpeedPIDControllerParameters
{
  public:
    /// Constructor
    explicit SpeedPIDControllerParameters(pid::PID* pid = nullptr ///< [in]  PID parameters
                                          )
        : pid_(pid){};

    /// Get PID parameters
    /// return     PID parameters pointer
    pid::PID* pid() const
    {
        return pid_;
    };

    /// Set PID
    void set_pid(pid::PID* pid ///< [in]   new PID
    )
    {
        pid_ = pid;
    };

  private:
    /// PID parameters
    pid::PID* pid_; ///< Speed PID
};

} // namespace motion_control

} // namespace cogip

/// @}
