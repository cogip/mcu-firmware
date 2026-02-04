// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    path_manager_filter
/// @{
/// @file
/// @brief      Path manager filter parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <cstddef>

namespace cogip {

namespace motion_control {

/// @brief Parameters for PathManagerFilter.
class PathManagerFilterParameters
{
  public:
    /// Constructor.
    explicit PathManagerFilterParameters(size_t max_waypoints = 32) : max_waypoints_(max_waypoints)
    {
    }

    /// Get maximum number of waypoints.
    size_t max_waypoints() const
    {
        return max_waypoints_;
    }

  private:
    size_t max_waypoints_; ///< Maximum number of waypoints in path
};

} // namespace motion_control

} // namespace cogip

/// @}
