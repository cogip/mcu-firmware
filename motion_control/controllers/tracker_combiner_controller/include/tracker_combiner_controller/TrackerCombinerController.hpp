// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    tracker_combiner_controller Tracker Combiner controller
/// @{
/// @file
/// @brief      Tracker Combiner controller
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "TrackerCombinerControllerIOKeys.hpp"
#include "TrackerCombinerControllerParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {

namespace motion_control {

/// @brief Tracker Combiner controller
///
/// This controller simply adds tracker velocity and feedback correction:
///   speed_order = tracker_velocity + feedback_correction
///
/// **Usage in chain:**
/// @code
/// ProfileTrackerController → tracker_velocity, tracking_error
///                               ↓
/// PosePIDController → input: tracking_error → output: feedback_correction
///                               ↓
/// TrackerCombinerController → speed_order = tracker + feedback
/// @endcode
class TrackerCombinerController
    : public Controller<TrackerCombinerControllerIOKeys, TrackerCombinerControllerParameters>
{
  public:
    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys
    /// @param parameters Reference to controller parameters (empty)
    /// @param name       Optional instance name for identification
    explicit TrackerCombinerController(const TrackerCombinerControllerIOKeys& keys,
                                       const TrackerCombinerControllerParameters& parameters,
                                       etl::string_view name = "")
        : Controller<TrackerCombinerControllerIOKeys, TrackerCombinerControllerParameters>(
              keys, parameters, name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "TrackerCombinerController";
    }

    /// @brief Read tracker_velocity and feedback_correction,
    ///        add them, and write to speed_order
    /// @param io Controller IO map
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
