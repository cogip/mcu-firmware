// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_path
/// @{
/// @file
/// @brief       Path class for waypoint list management
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "path/Pose.hpp"

#include <etl/vector.h>

namespace cogip {

namespace path {

/// @brief Path class that manages a list of waypoints.
///
/// This class provides methods to build and traverse a path of waypoints.
/// It is used by motion control filters (PathManagerFilter, PurePursuitFilter)
/// to navigate through multiple target poses.
class Path
{
  public:
    /// Maximum number of waypoints in the path
    static constexpr size_t MAX_WAYPOINTS = 32;

    /// Container type for waypoints
    using PathContainer = etl::vector<Pose, MAX_WAYPOINTS>;

    /// @brief Get the singleton instance.
    /// @return Reference to the unique Path instance
    static Path& instance();

    /// @brief Constructor.
    Path();

    /// @brief Reset the path (clear all waypoints).
    void reset();

    /// @brief Add a waypoint to the path.
    /// @param pose The waypoint to add
    /// @return true if added successfully, false if path is full
    bool add_point(const Pose& pose);

    /// @brief Add a waypoint from a Protobuf message.
    /// @param pb_pose The Protobuf message to read
    /// @return true if added successfully, false if path is full
    bool add_point_from_pb(const PB_PathPose& pb_pose);

    /// @brief Start path execution.
    void start();

    /// @brief Stop path execution.
    void stop();

    /// @brief Advance to the next waypoint.
    /// @return true if advanced to next waypoint, false if path is complete
    bool advance();

    /// @brief Get the current waypoint.
    /// @return Pointer to current waypoint, or nullptr if not available
    const Pose* current_pose() const;

    /// @brief Get a waypoint at a specific index.
    /// @param index The waypoint index
    /// @return Pointer to waypoint, or nullptr if index is invalid
    const Pose* waypoint_at(size_t index) const;

    /// @brief Check if path execution has started.
    /// @return true if started
    bool is_started() const;

    /// @brief Check if path execution is complete.
    /// @return true if all waypoints have been reached
    bool is_complete() const;

    /// @brief Check if we're at the last waypoint.
    /// @return true if at last waypoint
    bool is_last_waypoint() const;

    /// @brief Get current waypoint index.
    /// @return Current index in the path
    size_t current_index() const;

    /// @brief Get number of waypoints in path.
    /// @return Number of waypoints
    size_t size() const;

    /// @brief Check if path is empty.
    /// @return true if no waypoints
    bool empty() const;

    /// @brief Get all waypoints (for PurePursuit).
    /// @return Reference to waypoints vector
    const PathContainer& waypoints() const;

  private:
    PathContainer waypoints_; ///< List of waypoints
    size_t current_index_;    ///< Current waypoint index
    bool started_;            ///< Path execution started
};

} // namespace path

} // namespace cogip

/// @}
