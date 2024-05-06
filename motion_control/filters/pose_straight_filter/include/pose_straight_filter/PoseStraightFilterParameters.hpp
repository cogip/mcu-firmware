// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    pose_straight_filter Pose straight filter parameters
/// @{
/// @file
/// @brief      Movements switch thresholds
/// @author     Eric Courtois <eric.courtois@gmail.com>
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Movements switch thresholds
class PoseStraightFilterParameters {
public:
    /// Constructor
    PoseStraightFilterParameters(
        double angular_threshold = 0.0,  ///< [in]  see angular_threshold_
        double linear_threshold = 0.0,   ///< [in]  see linear_threshold_
        double angular_intermediate_threshold = 0.0,  ///< [in]  see angular_threshold_
        double angular_deceleration = 0.0,  ///< [in]  see angular_deceleration_threshold_
        double linear_deceleration = 0.0,   ///< [in]  see linear_deceleration_threshold_
        bool bypass_final_orientation = false  ///< [in] bypass final orientation
    ) :
    angular_threshold_(angular_threshold),
    linear_threshold_(linear_threshold),
    angular_intermediate_threshold_(angular_intermediate_threshold),
    angular_deceleration_(angular_deceleration),
    linear_deceleration_(linear_deceleration),
    bypass_final_orientation_(bypass_final_orientation) {};

    /// Get angular threshold
    /// return angular threshold
    double angular_threshold() const { return angular_threshold_; };

    /// Set angular threshold
    void set_angular_threshold(
        double angular_threshold                            ///< [in]   angular threshold
        ) { angular_threshold_ = angular_threshold; };

    /// Get linear threshold
    /// return Linear threshold
    double linear_threshold() const { return linear_threshold_; };

    /// Get angular intermediate threshold
    /// return angular intermediate threshold
    double angular_intermediate_threshold() const { return angular_intermediate_threshold_; };

    /// Set angular threshold
    void set_intermediate_angular_threshold(
        double angular_intermediate_threshold                            ///< [in]   intermediate angular threshold
        ) { angular_intermediate_threshold_ = angular_intermediate_threshold; };

    /// Get angular deceleration
    /// return Angular deceleration
    double angular_deceleration() const { return angular_deceleration_; };

    /// Get linear deceleration
    /// return Linear deceleration
    double linear_deceleration() const { return linear_deceleration_; };

    /// Set linear threshold
    void set_linear_threshold(
        double linear_threshold                             ///< [in]   linear threshold
        ) { linear_threshold_ = linear_threshold; };

    /// Set angular deceleration
    void set_angular_deceleration(
        double angular_deceleration                         ///< [in]   angular deceleration
        ) { angular_deceleration_ = angular_deceleration; };

    /// Set linear deceleration
    void set_linear_deceleration(
        double linear_deceleration                          ///< [in]   linear deceleration
        ) { linear_deceleration_ = linear_deceleration; };

    /// Return final orientation bypass
    bool bypass_final_orientation() const { return bypass_final_orientation_; };

    /// Activate final orientation bypass
    void bypass_final_orientation_on() { bypass_final_orientation_ = true; };

    /// Activate final orientation bypass
    void bypass_final_orientation_off() { bypass_final_orientation_ = false; };

private:
    /// the robot turns on itself until the angle error is lower than this threshold
    double angular_threshold_;

    /// the robot has reach the point when the linear error is lower than this threshold
    double linear_threshold_;

    /// the robot turns on itself until the angle error is lower than this threshold to reach its final destination
    double angular_intermediate_threshold_;

    /// angular deceleration
    double angular_deceleration_;

    /// linear deceleration
    double linear_deceleration_;

    /// bypass final orientation
    bool bypass_final_orientation_;
};

} // namespace motion_control

} // namespace cogip

/// @}
