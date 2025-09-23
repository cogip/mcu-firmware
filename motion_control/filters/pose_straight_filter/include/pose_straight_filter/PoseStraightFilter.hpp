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
#include "motion_control_common/ControllersIO.hpp"
#include "PoseStraightFilterParameters.hpp"
#include "PoseStraightFilterIOKeys.hpp"

namespace cogip {

namespace motion_control {

// Motion states
enum class PoseStraightFilterState {
    ROTATE_TO_DIRECTION,
    MOVE_TO_POSITION,
    ROTATE_TO_FINAL_ANGLE,
    FINISHED
};

/// @brief Motion broken into rotate‐move‐rotate steps according to state machine.
///        Reads full current and target poses, speeds, and reverse permission from IO,
///        then computes filtered linear and angular speeds plus pose‐reached flag.
class PoseStraightFilter
    : public Controller<PoseStraightFilterIOKeys, PoseStraightFilterParameters>
{
public:
    /// @brief Constructor.
    /// @param keys       Reference to a POD containing all input and output key names.
    /// @param parameters Reference to movement switch thresholds.
    explicit PoseStraightFilter(
        const PoseStraightFilterIOKeys&         keys,
        const PoseStraightFilterParameters&     parameters
    )
        : Controller<PoseStraightFilterIOKeys, PoseStraightFilterParameters>(keys, parameters)
    {
        current_state_ = PoseStraightFilterState::FINISHED;
    }

    /// @brief Evaluate state machine and compute errors, filtered speeds, and reached status.
    /// @param io Shared ControllersIO containing inputs and receiving outputs.
    void execute(ControllersIO& io) override;

    /// @brief Reset internal state machine to initial rotation state.
    void reset_current_state() { current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION; }

    /// @brief Force state machine to finished state.
    void force_finished_state() { current_state_ = PoseStraightFilterState::FINISHED; }

private:
    PoseStraightFilterState current_state_;
};

} // namespace motion_control

} // namespace cogip

/// @}
