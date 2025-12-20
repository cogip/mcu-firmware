// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_error_filter
/// @{
/// @file
/// @brief      Pose error filter parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Mode for PoseErrorFilter
enum class PoseErrorFilterMode {
    LINEAR, ///< Compute Euclidean distance from (x, y) coordinates
    ANGULAR ///< Compute angle difference from orientation
};

/// @brief Parameters for PoseErrorFilter.
class PoseErrorFilterParameters
{
  public:
    /// Constructor.
    explicit PoseErrorFilterParameters(
        PoseErrorFilterMode mode,           ///< [in] filter mode
        float pose_reached_threshold = 0.0f ///< [in] threshold for pose reached
        )
        : mode_(mode), pose_reached_threshold_(pose_reached_threshold)
    {
    }

    /// Get filter mode.
    PoseErrorFilterMode mode() const
    {
        return mode_;
    }

    /// Get pose reached threshold.
    float pose_reached_threshold() const
    {
        return pose_reached_threshold_;
    }

    /// Set pose reached threshold.
    void set_pose_reached_threshold(float threshold)
    {
        pose_reached_threshold_ = threshold;
    }

  private:
    PoseErrorFilterMode mode_;     ///< Filter mode (linear or angular)
    float pose_reached_threshold_; ///< Threshold for pose reached detection
};

} // namespace motion_control

} // namespace cogip

/// @}
