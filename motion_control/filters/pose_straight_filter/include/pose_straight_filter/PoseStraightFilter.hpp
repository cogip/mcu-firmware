// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_straight_filter Pose straight filter
/// @{
/// @file
/// @brief      Breaks down a movement into a straight trajectory
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/Controller.hpp"
#include "PoseStraightFilterParameters.hpp"

namespace cogip {

namespace motion_control {

// Motion states
enum class PoseStraightFilterState {
    ROTATE_TO_DIRECTION,
    MOVE_TO_POSITION,
    ROTATE_TO_FINAL_ANGLE,
    FINISHED
};

/// Breaks down a movement into a straight trajectory.
///        The robot first orients itself towards the point to be reached, then goes towards this point in a straight line.
/// Input 0-2:  current pose
/// Input 3-5:  target pose
/// Input 6-7:  current speed
/// Input 8-9:  target speed
/// Input 10:   allow reverse
/// Output 0:   linear pose error
/// Output 1:   linear current speed
/// Output 2:   linear target speed
/// Output 3:   angular pose error
/// Output 4:   angular current speed
/// Output 5:   angular target speed
/// Output 5:   pose reached
class PoseStraightFilter : public Controller<11, 9, PoseStraightFilterParameters> {
public:
    /// Constructor
    explicit PoseStraightFilter(
        PoseStraightFilterParameters *parameters    ///< [in]  Movements switch thresholds. See PoseStraightFilterParameters.
        ) : Controller(parameters) { current_state_ = PoseStraightFilterState::FINISHED; };

    /// Breaks down a movement into a straight trajectory according to movements switch thresholds.
    void execute() override;

    /// Reset state machine current state
    void reset_current_state() { current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION; };

    /// Force state machine to finished state
    void force_finished_state() { current_state_ = PoseStraightFilterState::FINISHED; };

private:
    /// Current motion state
    PoseStraightFilterState current_state_;
};

} // namespace motion_control

} // namespace cogip

/// @}
