// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup
/// @{
/// @file
/// @brief      PurePursuit controller parameters
/// @author

#pragma once

#include "parameter/ParameterInterface.hpp"

using namespace cogip::parameter;

namespace cogip {

namespace motion_control {

/// @brief Parameters for PurePursuitControllerParameters.
class PurePursuitControllerParameters
{
  public:
    /// @brief Constructor with all parameters.
    /// @param linear_kp                 Linear proportional gain
    /// @param angular_kp                Angular proportional gain
    /// @param lookahead_distance        Lookahead distance (mm)
    /// @param final_lookahead_distance  Max extension past last waypoint on virtual segment (mm)
    /// @param max_linear_speed          Maximum linear speed (mm/period)
    /// @param max_angular_speed         Maximum angular speed (deg/period)
    /// @param linear_threshold          Tolerance to consider path complete (mm)
    /// @param angular_threshold         Tolerance for final orientation (deg)
    explicit PurePursuitControllerParameters(const ParameterInterface<float>& linear_kp,
                                             const ParameterInterface<float>& angular_kp,
                                             const ParameterInterface<float>& lookahead_distance,
                                             const ParameterInterface<float>& final_lookahead_distance,
                                             const ParameterInterface<float>& max_linear_speed,
                                             const ParameterInterface<float>& max_angular_speed,
                                             const ParameterInterface<float>& linear_threshold,
                                             const ParameterInterface<float>& angular_threshold)
        : linear_kp(linear_kp), angular_kp(angular_kp), lookahead_distance(lookahead_distance),
          final_lookahead_distance(final_lookahead_distance), max_linear_speed(max_linear_speed),
          max_angular_speed(max_angular_speed), linear_threshold(linear_threshold),
          angular_threshold(angular_threshold)
    {
    }

    /// Read-only parameters. Each parameter has its own getter.
    const ParameterInterface<float>& linear_kp;
    const ParameterInterface<float>& angular_kp;
    const ParameterInterface<float>& lookahead_distance;
    const ParameterInterface<float>& final_lookahead_distance;
    const ParameterInterface<float>& max_linear_speed;
    const ParameterInterface<float>& max_angular_speed;
    const ParameterInterface<float>& linear_threshold;
    const ParameterInterface<float>& angular_threshold;
};

} // namespace motion_control

} // namespace cogip

/// @}
