// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    profile_feedforward_controller Profile Feedforward controller
/// @{
/// @file
/// @brief      Profile Feedforward controller with trapezoidal velocity profile
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "ProfileFeedforwardControllerIOKeys.hpp"
#include "ProfileFeedforwardControllerParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include "motion_control_common/TrapezoidalProfile.hpp"

namespace cogip {

namespace motion_control {

/// @brief Profile Feedforward controller
///
/// This controller generates an optimal trapezoidal velocity profile and outputs:
/// 1. **Feedforward velocity**: theoretical velocity from the profile
/// 2. **Tracking error**: difference between actual and theoretical remaining distance
///
/// **Workflow:**
/// - When `new_target` flag is set:
///   - Generate trapezoidal profile with initial `pose_error` (total distance)
///   - Reset period counter
/// - Every cycle:
///   - Compute feedforward velocity from profile
///   - Compute theoretical remaining distance from profile
///   - Read actual remaining distance (pose_error updated by odometry)
///   - Compute tracking error = actual_remaining - theoretical_remaining
///
/// **Usage:**
/// The tracking error output should be fed to a PosePIDController for correction.
/// The feedforward velocity should be combined with PID output (addition).
///
/// @code
/// // Example chain:
/// ProfileFeedforwardController → outputs feedforward + tracking_error
///                               ↓
/// PosePIDController → input: tracking_error → output: correction
///                               ↓
/// FeedforwardCombiner → speed_order = feedforward + correction
/// @endcode
class ProfileFeedforwardController
    : public Controller<ProfileFeedforwardControllerIOKeys, ProfileFeedforwardControllerParameters>
{
  public:
    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys
    /// @param parameters Reference to controller parameters
    explicit ProfileFeedforwardController(const ProfileFeedforwardControllerIOKeys& keys,
                                          const ProfileFeedforwardControllerParameters& parameters);

    /// @brief Execute profile feedforward computation
    ///
    /// 1. Check if new_target flag is set
    ///    - If yes: generate new trapezoidal profile with pose_error as target distance
    /// 2. Compute feedforward velocity from profile
    /// 3. Compute theoretical remaining distance from profile
    /// 4. Read actual remaining distance (pose_error)
    /// 5. Compute tracking error = actual - theoretical
    /// 6. Write outputs: feedforward_velocity, tracking_error
    /// 7. Increment period counter
    ///
    /// @param io Controller IO map for reading inputs and writing outputs
    void execute(ControllersIO& io) override;

  private:
    TrapezoidalProfile profile_; ///< Trapezoidal velocity profile generator
    uint32_t period_;            ///< Current period counter
};

} // namespace motion_control

} // namespace cogip

/// @}
