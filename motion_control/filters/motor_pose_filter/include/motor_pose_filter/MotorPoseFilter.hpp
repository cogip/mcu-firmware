// Copyright (C) 2024 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motor_pose_filter Pose straight filter
/// @{
/// @file
/// @brief      Breaks down a movement into a straight trajectory
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/Controller.hpp"
#include "MotorPoseFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// Breaks down a movement into a straight trajectory.
///        The robot first orients itself towards the point to be reached, then goes towards this point in a straight line.
/// Input 0:    current pose
/// Input 1:    target pose
/// Input 2:    current speed
/// Input 3:    target speed
/// Output 0:   pose error
/// Output 1:   current speed
/// Output 2:   target speed
/// Output 3:   speed filter flag
/// Output 4:   pose reached
class MotorPoseFilter : public Controller<5, 5, MotorPoseFilterParameters> {
public:
    /// Constructor
    explicit MotorPoseFilter(
        MotorPoseFilterParameters *parameters    ///< [in]  Movements switch thresholds. See MotorPoseFilterParameters.
        ) : Controller(parameters) { };

    /// Breaks down a movement into a straight trajectory according to movements switch thresholds.
    void execute() override;
};

} // namespace motion_control

} // namespace cogip

/// @}
