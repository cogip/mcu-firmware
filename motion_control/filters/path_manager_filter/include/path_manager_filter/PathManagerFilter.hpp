// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    path_manager_filter
/// @{
/// @file
/// @brief      Path manager filter class declaration
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include "path/Path.hpp"

#include "PathManagerFilterIOKeys.hpp"
#include "PathManagerFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// @brief Path manager filter that processes a list of waypoints.
///
/// This filter reads a Path object from ControllersIO and advances
/// to the next waypoint when the current pose is reached. The Path
/// is stored in PlatformEngine and passed via IOKey.
///
/// The filter uses the is_intermediate() flag on each waypoint to determine
/// behavior:
/// - Intermediate waypoints: advance to next when reached
/// - Final waypoint (not intermediate): signal path complete when reached
///
/// The filter emits target_pose coordinates for downstream filters
/// (PoseStraightFilter) and signals path_complete when the final waypoint
/// is reached.
class PathManagerFilter : public Controller<PathManagerFilterIOKeys, PathManagerFilterParameters>
{
  public:
    /// @brief Constructor.
    explicit PathManagerFilter(const PathManagerFilterIOKeys& keys,
                               PathManagerFilterParameters& parameters, etl::string_view name = "")
        : Controller<PathManagerFilterIOKeys, PathManagerFilterParameters>(keys, parameters, name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "PathManagerFilter";
    }

    /// Execute the path manager filter.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
