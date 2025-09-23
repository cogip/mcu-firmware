// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motor_pose_filter Motor pose filter
/// @{
/// @file
/// @brief      Filter one motor positionning
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "MotorPoseFilterIOKeys.hpp"
#include "MotorPoseFilterParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {
namespace motion_control {

/// @brief Filter one motor positionning.
class MotorPoseFilter : public Controller<MotorPoseFilterIOKeys, MotorPoseFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to IO keys.
    /// @param parameters Reference to parameters.
    explicit MotorPoseFilter(const MotorPoseFilterIOKeys& keys,
                             const MotorPoseFilterParameters& parameters)
        : Controller<MotorPoseFilterIOKeys, MotorPoseFilterParameters>(keys, parameters)
    {
    }

    /// @brief Compute pose error and decide filtered speed and pose reached
    /// status.
    /// @param io Shared controllers IOs.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
