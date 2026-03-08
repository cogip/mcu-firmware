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
class PoseStraightFilterParameters
{
  public:
    /// Constructor
    explicit PoseStraightFilterParameters(
        float angular_threshold = 0.0,              ///< [in]  see angular_threshold_
        float linear_threshold = 0.0,               ///< [in]  see linear_threshold_
        float angular_intermediate_threshold = 0.0, ///< [in]  see angular_threshold_
        float angular_deceleration = 0.0,           ///< [in]  see angular_deceleration_threshold_
        float linear_deceleration = 0.0,            ///< [in]  see linear_deceleration_threshold_
        bool bypass_final_orientation = false,      ///< [in] bypass final orientation
        bool use_angle_continuity = false           ///< [in] use angle continuity enforcement
        )
        : angular_threshold_(angular_threshold), linear_threshold_(linear_threshold),
          angular_intermediate_threshold_(angular_intermediate_threshold),
          angular_deceleration_(angular_deceleration), linear_deceleration_(linear_deceleration),
          bypass_final_orientation_(bypass_final_orientation),
          use_angle_continuity_(use_angle_continuity){};

    /// Get angular threshold
    /// return angular threshold
    float angular_threshold() const
    {
        return angular_threshold_;
    };

    /// Set angular threshold
    void set_angular_threshold(float angular_threshold ///< [in]   angular threshold
    )
    {
        angular_threshold_ = angular_threshold;
    };

    /// Get linear threshold
    /// return Linear threshold
    float linear_threshold() const
    {
        return linear_threshold_;
    };

    /// Get angular intermediate threshold
    /// return angular intermediate threshold
    float angular_intermediate_threshold() const
    {
        return angular_intermediate_threshold_;
    };

    /// Set angular threshold
    void
    set_intermediate_angular_threshold(float angular_intermediate_threshold ///< [in]   intermediate
                                                                            ///< angular threshold
    )
    {
        angular_intermediate_threshold_ = angular_intermediate_threshold;
    };

    /// Get angular deceleration
    /// return Angular deceleration
    float angular_deceleration() const
    {
        return angular_deceleration_;
    };

    /// Get linear deceleration
    /// return Linear deceleration
    float linear_deceleration() const
    {
        return linear_deceleration_;
    };

    /// Set linear threshold
    void set_linear_threshold(float linear_threshold ///< [in]   linear threshold
    )
    {
        linear_threshold_ = linear_threshold;
    };

    /// Set angular deceleration
    void set_angular_deceleration(float angular_deceleration ///< [in]   angular deceleration
    )
    {
        angular_deceleration_ = angular_deceleration;
    };

    /// Set linear deceleration
    void set_linear_deceleration(float linear_deceleration ///< [in]   linear deceleration
    )
    {
        linear_deceleration_ = linear_deceleration;
    };

    /// Return final orientation bypass
    bool bypass_final_orientation() const
    {
        return bypass_final_orientation_;
    };

    /// Activate final orientation bypass
    void bypass_final_orientation_on()
    {
        bypass_final_orientation_ = true;
    };

    /// Activate final orientation bypass
    void bypass_final_orientation_off()
    {
        bypass_final_orientation_ = false;
    };

    /// Return angle continuity mode
    bool use_angle_continuity() const
    {
        return use_angle_continuity_;
    };

    /// Enable angle continuity enforcement
    void use_angle_continuity_on()
    {
        use_angle_continuity_ = true;
    };

    /// Disable angle continuity enforcement
    void use_angle_continuity_off()
    {
        use_angle_continuity_ = false;
    };

  private:
    /// the robot turns on itself until the angle error is lower than this
    /// threshold
    float angular_threshold_;

    /// the robot has reach the point when the linear error is lower than this
    /// threshold
    float linear_threshold_;

    /// the robot turns on itself until the angle error is lower than this
    /// threshold to reach its final destination
    float angular_intermediate_threshold_;

    /// angular deceleration
    float angular_deceleration_;

    /// linear deceleration
    float linear_deceleration_;

    /// bypass final orientation
    bool bypass_final_orientation_;

    /// use angle continuity enforcement (for ProfileTracker)
    bool use_angle_continuity_;
};

} // namespace motion_control

} // namespace cogip

/// @}
