// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    profile_feedforward_controller Profile Feedforward controller parameters
/// @{
/// @file
/// @brief      Profile Feedforward controller parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Profile Feedforward controller parameters
class ProfileFeedforwardControllerParameters
{
  public:
    /// Constructor
    explicit ProfileFeedforwardControllerParameters(
        float max_speed = 10.0f,     ///< [in] Maximum speed (mm/period or rad/period)
        float acceleration = 1.0f,   ///< [in] Acceleration (mm/period² or rad/period²)
        float deceleration = 1.0f,   ///< [in] Deceleration (mm/period² or rad/period²)
        bool must_stop_at_end = true ///< [in] Stop at target (true) or continue (false)
        )
        : max_speed_(max_speed), acceleration_(acceleration), deceleration_(deceleration),
          must_stop_at_end_(must_stop_at_end)
    {
    }

    /// Get maximum speed
    float max_speed() const
    {
        return max_speed_;
    }

    /// Set maximum speed
    void set_max_speed(float max_speed)
    {
        max_speed_ = max_speed;
    }

    /// Get acceleration
    float acceleration() const
    {
        return acceleration_;
    }

    /// Set acceleration
    void set_acceleration(float acceleration)
    {
        acceleration_ = acceleration;
    }

    /// Get deceleration
    float deceleration() const
    {
        return deceleration_;
    }

    /// Set deceleration
    void set_deceleration(float deceleration)
    {
        deceleration_ = deceleration;
    }

    /// Get must_stop_at_end flag
    bool must_stop_at_end() const
    {
        return must_stop_at_end_;
    }

    /// Set must_stop_at_end flag
    void set_must_stop_at_end(bool must_stop_at_end)
    {
        must_stop_at_end_ = must_stop_at_end;
    }

  private:
    float max_speed_;       ///< Maximum velocity limit
    float acceleration_;    ///< Maximum acceleration
    float deceleration_;    ///< Maximum deceleration
    bool must_stop_at_end_; ///< Stop at end or continue
};

} // namespace motion_control

} // namespace cogip

/// @}
