// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    feedforward_combiner_controller Feedforward Combiner controller
/// @{
/// @file
/// @brief      Feedforward Combiner controller
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "FeedforwardCombinerControllerIOKeys.hpp"
#include "FeedforwardCombinerControllerParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {

namespace motion_control {

/// @brief Feedforward Combiner controller
///
/// This controller simply adds feedforward velocity and feedback correction:
///   speed_order = feedforward_velocity + feedback_correction
///
/// **Usage in chain:**
/// @code
/// ProfileFeedforwardController → feedforward_velocity, tracking_error
///                               ↓
/// PosePIDController → input: tracking_error → output: feedback_correction
///                               ↓
/// FeedforwardCombinerController → speed_order = feedforward + feedback
/// @endcode
class FeedforwardCombinerController : public Controller<FeedforwardCombinerControllerIOKeys,
                                                        FeedforwardCombinerControllerParameters>
{
  public:
    /// @brief Constructor
    /// @param keys       Reference to a POD containing all controller keys
    /// @param parameters Reference to controller parameters (empty)
    /// @param name       Optional instance name for identification
    explicit FeedforwardCombinerController(
        const FeedforwardCombinerControllerIOKeys& keys,
        const FeedforwardCombinerControllerParameters& parameters, etl::string_view name = "")
        : Controller<FeedforwardCombinerControllerIOKeys, FeedforwardCombinerControllerParameters>(
              keys, parameters, name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "FeedforwardCombinerController";
    }

    /// @brief Read feedforward_velocity and feedback_correction,
    ///        add them, and write to speed_order
    /// @param io Controller IO map
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
