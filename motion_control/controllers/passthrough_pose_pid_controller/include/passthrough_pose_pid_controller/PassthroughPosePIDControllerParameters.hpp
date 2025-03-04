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
class PassthroughPosePIDControllerParameters {
public:
    /// Constructor
    PassthroughPosePIDControllerParameters(
        float target_speed = 0.0,          ///< [in] PID parameters
        bool signed_target_speed = true     ///< [in] target speed signed flag
    ) : target_speed_(target_speed), signed_target_speed_(signed_target_speed) {};

    /// Get target speed parameter
    /// return     target speed
    float target_speed() const { return target_speed_; };

    void set_target_speed(
        float target_speed
        ) { target_speed_ = target_speed; }

    /// Get force target speed flag
    /// return      force target speed flag
    bool signed_target_speed() const { return signed_target_speed_; };

    void set_signed_target_speed(
        bool signed_target_speed
        ) { signed_target_speed_ = signed_target_speed; }

private:
    /// Target speed
    float target_speed_;

    /// Force target speed flag. If set, speed_order is forced to target_speed
    bool signed_target_speed_;
};

} // namespace motion_control

} // namespace cogip

/// @}
