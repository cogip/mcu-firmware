// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    target_change_detector Target Change Detector
/// @{
/// @file
/// @brief      Target Change Detector controller
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "TargetChangeDetectorIOKeys.hpp"
#include "TargetChangeDetectorParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {

namespace motion_control {

/// @brief Target Change Detector controller
///
/// This controller detects when the target pose changes significantly and sets
/// a flag to trigger profile regeneration in downstream controllers like
/// ProfileFeedforwardController.
///
/// **Workflow:**
/// - Read current target pose from IO
/// - Compare with previously stored target pose
/// - If change exceeds threshold, set new_target flag to true
/// - Store current target for next comparison
///
/// **Usage:**
/// Place this controller before ProfileFeedforwardController in the chain.
///
/// @code
/// // Example chain:
/// PoseStraightFilter → outputs pose_error
///                     ↓
/// TargetChangeDetector → outputs new_target flag
///                     ↓
/// ProfileFeedforwardController → uses new_target to regenerate profile
/// @endcode
class TargetChangeDetector
    : public Controller<TargetChangeDetectorIOKeys, TargetChangeDetectorParameters>
{
  public:
    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys
    /// @param parameters Reference to controller parameters
    /// @param name       Optional instance name for identification
    explicit TargetChangeDetector(const TargetChangeDetectorIOKeys& keys,
                                  const TargetChangeDetectorParameters& parameters,
                                  etl::string_view name = "");

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "TargetChangeDetector";
    }

    /// @brief Execute target change detection
    ///
    /// 1. Read current target pose from IO
    /// 2. Compare with previously stored target pose
    /// 3. If change exceeds threshold, set new_target flag to true
    /// 4. Store current target for next comparison
    ///
    /// @param io Controller IO map for reading inputs and writing outputs
    void execute(ControllersIO& io) override;

  private:
    float previous_target_x_; ///< Previously stored target X coordinate
    float previous_target_y_; ///< Previously stored target Y coordinate
    float previous_target_O_; ///< Previously stored target orientation
    int previous_state_;      ///< Previously stored state (for state transition mode)
    bool first_run_;          ///< True on first execution
};

} // namespace motion_control

} // namespace cogip

/// @}
