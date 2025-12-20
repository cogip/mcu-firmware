// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    tuning_pose_reached_filter Tuning pose reached filter
/// @{
/// @file
/// @brief      Simple filter for tuning mode that sets pose_reached when profile is complete
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "TuningPoseReachedFilterIOKeys.hpp"
#include "motion_control_common/BaseController.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {
namespace motion_control {

/// @brief Simple filter for tuning mode that sets pose_reached when profile is complete.
///        Used in speed tuning chains where we just want to know when the velocity
///        profile has finished, without complex pose error checking.
class TuningPoseReachedFilter : public BaseController
{
  public:
    /// @brief Constructor.
    /// @param keys Reference to a POD containing all input and output key names.
    /// @param name Optional instance name for identification.
    explicit TuningPoseReachedFilter(const TuningPoseReachedFilterIOKeys& keys,
                                     etl::string_view name = "")
        : BaseController(name), keys_(keys)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "TuningPoseReachedFilter";
    }

    /// @brief Check profile_complete and set pose_reached accordingly.
    /// @param io Shared ControllersIO containing inputs and receiving outputs.
    void execute(ControllersIO& io) override;

  private:
    const TuningPoseReachedFilterIOKeys& keys_; ///< IO keys
};

} // namespace motion_control

} // namespace cogip

/// @}
