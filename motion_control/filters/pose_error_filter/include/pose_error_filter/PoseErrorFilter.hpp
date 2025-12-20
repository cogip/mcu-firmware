// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_error_filter
/// @{
/// @file
/// @brief      Pose error filter class declaration
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

#include "PoseErrorFilterIOKeys.hpp"
#include "PoseErrorFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// @brief Simple pose error filter that computes error from target and current pose.
///
/// This filter computes the pose error without a state machine:
/// - LINEAR mode: error = Euclidean distance between target and current (x, y)
/// - ANGULAR mode: error = angle difference between target and current orientation
///
/// It also detects target changes and sets a recompute flag for downstream
/// profile generators.
class PoseErrorFilter : public Controller<PoseErrorFilterIOKeys, PoseErrorFilterParameters>
{
  public:
    /// @brief Constructor.
    explicit PoseErrorFilter(const PoseErrorFilterIOKeys& keys,
                             PoseErrorFilterParameters& parameters, etl::string_view name = "")
        : Controller<PoseErrorFilterIOKeys, PoseErrorFilterParameters>(keys, parameters, name),
          prev_target_x_(0.0f), prev_target_y_(0.0f), prev_target_O_(0.0f), first_run_(true)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "PoseErrorFilter";
    }

    /// Execute the pose error filter.
    void execute(ControllersIO& io) override;

  private:
    /// Execute linear mode: compute Euclidean distance.
    /// @param io Controllers IO
    /// @param[out] pose_error Computed pose error
    /// @param[out] target_changed True if target changed
    void execute_linear(ControllersIO& io, float& pose_error, bool& target_changed);

    /// Execute angular mode: compute angle difference.
    /// @param io Controllers IO
    /// @param[out] pose_error Computed pose error
    /// @param[out] target_changed True if target changed
    void execute_angular(ControllersIO& io, float& pose_error, bool& target_changed);

    float prev_target_x_; ///< Previous target X for change detection
    float prev_target_y_; ///< Previous target Y for change detection
    float prev_target_O_; ///< Previous target O for change detection
    bool first_run_;      ///< First run flag
};

} // namespace motion_control

} // namespace cogip

/// @}
