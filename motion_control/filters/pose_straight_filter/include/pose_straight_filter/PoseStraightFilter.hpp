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
#include "PoseStraightFilterIOKeys.hpp"
#include "PoseStraightFilterParameters.hpp"
#include "cogip_defs/Polar.hpp"
#include "cogip_defs/Pose.hpp"
#include "etl/numeric.h"
#include "log.h"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include "trigonometry.h"

namespace cogip {

namespace motion_control {

// Motion states
enum class PoseStraightFilterState {
    ROTATE_TO_DIRECTION,
    MOVE_TO_POSITION,
    ROTATE_TO_FINAL_ANGLE,
    FINISHED
};

/// @brief Motion broken into rotate‐move‐rotate steps according to state
/// machine.
///        Reads full current and target poses, speeds, and reverse permission
///        from IO, then computes filtered linear and angular speeds plus
///        pose‐reached flag.
class PoseStraightFilter : public Controller<PoseStraightFilterIOKeys, PoseStraightFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to a POD containing all input and output key
    /// names.
    /// @param parameters Reference to movement switch thresholds.
    /// @param name       Optional instance name for identification.
    explicit PoseStraightFilter(const PoseStraightFilterIOKeys& keys,
                                const PoseStraightFilterParameters& parameters,
                                etl::string_view name = "")
        : Controller<PoseStraightFilterIOKeys, PoseStraightFilterParameters>(keys, parameters,
                                                                             name),
          current_state_(PoseStraightFilterState::FINISHED),
          prev_target_(etl::numeric_limits<int32_t>::max(), etl::numeric_limits<int32_t>::max(),
                       etl::numeric_limits<int32_t>::max()),
          start_pose_(etl::numeric_limits<int32_t>::max(), etl::numeric_limits<int32_t>::max(),
                      etl::numeric_limits<int32_t>::max()),
          logged_finished_(false), prev_angular_error_rotate_(0.0f), prev_angular_error_final_(0.0f)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "PoseStraightFilter";
    }

    /// @brief Evaluate state machine and compute errors, filtered speeds, and
    /// reached status.
    /// @param io Shared ControllersIO containing inputs and receiving outputs.
    void execute(ControllersIO& io) override;

    /// @brief Reset all internal state for new target.
    /// Called when changing target to reinitialize state machine and tracking variables.
    void reset() override
    {
        current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION;
        prev_target_ = cogip_defs::Pose(etl::numeric_limits<int32_t>::max(),
                                        etl::numeric_limits<int32_t>::max(),
                                        etl::numeric_limits<int32_t>::max());
        start_pose_ = cogip_defs::Pose(etl::numeric_limits<int32_t>::max(),
                                       etl::numeric_limits<int32_t>::max(),
                                       etl::numeric_limits<int32_t>::max());
        logged_finished_ = false;
        prev_angular_error_rotate_ = 0.0f;
        prev_angular_error_final_ = 0.0f;
        locked_reverse_ = false;
    }

    /// @brief Reset internal state machine to initial rotation state.
    void reset_current_state()
    {
        current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION;
    }

    /// @brief Force state machine to finished state.
    void force_finished_state()
    {
        current_state_ = PoseStraightFilterState::FINISHED;
    }

  private:
    PoseStraightFilterState current_state_;
    cogip_defs::Pose prev_target_;          ///< Previous target for change detection
    cogip_defs::Pose start_pose_;           ///< Start pose for anti-drift correction
    bool logged_finished_;                  ///< Flag to log FINISHED only once per target
    float prev_angular_error_rotate_;       ///< Previous angular error in ROTATE_TO_DIRECTION
    float prev_angular_error_final_;        ///< Previous angular error in ROTATE_TO_FINAL_ANGLE
    bool bypass_final_orientation_ = false; ///< Cached bypass_final_orientation from IO
    bool locked_reverse_ = false;           ///< Locked reverse decision for current waypoint

    /// @brief Handle ROTATE_TO_DIRECTION state
    void rotate_to_direction(ControllersIO& io, cogip_defs::Polar& pos_err,
                             const cogip_defs::Pose& current_pose,
                             const cogip_defs::Pose& start_pose, bool& linear_recompute_profile,
                             bool& angular_recompute_profile);

    /// @brief Handle MOVE_TO_POSITION state
    void move_to_position(ControllersIO& io, cogip_defs::Polar& pos_err,
                          const cogip_defs::Pose& current_pose, cogip_defs::Pose& target_pose,
                          bool& linear_recompute_profile, bool& angular_recompute_profile);

    /// @brief Handle ROTATE_TO_FINAL_ANGLE state
    void rotate_to_final_angle(ControllersIO& io, cogip_defs::Polar& pos_err,
                               const cogip_defs::Pose& current_pose,
                               const cogip_defs::Pose& target_pose, bool& linear_recompute_profile,
                               bool& angular_recompute_profile);

    /// @brief Handle FINISHED state
    void finished(cogip_defs::Polar& pos_err, const cogip_defs::Pose& current_pose,
                  cogip_defs::Pose& target_pose);

    /// @brief Calculate longitudinal position error projected onto robot's axis
    /// @param current_x Current X position
    /// @param current_y Current Y position
    /// @param current_angle_deg Current orientation in degrees
    /// @param target_x Target X position
    /// @param target_y Target Y position
    /// @return Longitudinal error (positive = robot is ahead of target)
    inline float compute_longitudinal_error(float current_x, float current_y,
                                            float current_angle_deg, float target_x,
                                            float target_y) const
    {
        float dx = current_x - target_x;
        float dy = current_y - target_y;
        float current_angle_rad = DEG2RAD(current_angle_deg);
        return dx * std::cos(current_angle_rad) + dy * std::sin(current_angle_rad);
    }
};

} // namespace motion_control

} // namespace cogip

/// @}
