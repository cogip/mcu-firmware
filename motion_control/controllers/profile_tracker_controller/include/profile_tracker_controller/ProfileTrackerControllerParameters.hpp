// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    profile_tracker_controller Profile Tracker controller parameters
/// @{
/// @file
/// @brief      Profile Tracker controller parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// Profile Tracker controller parameters
class ProfileTrackerControllerParameters
{
  public:
    /// Constructor
    explicit ProfileTrackerControllerParameters(
        float max_speed = 10.0f,      ///< [in] Maximum speed (mm/period or rad/period)
        float acceleration = 1.0f,    ///< [in] Acceleration (mm/period² or rad/period²)
        float deceleration = 1.0f,    ///< [in] Deceleration (mm/period² or rad/period²)
        bool must_stop_at_end = true, ///< [in] Stop at target (true) or continue (false)
        uint16_t period_increment =
            1, ///< [in] Period increment per execute (for throttled controllers)
        bool speed_mode =
            false ///< [in] Speed mode: use target_speed + duration_periods instead of pose_error
        )
        : max_speed_(max_speed), acceleration_(acceleration), deceleration_(deceleration),
          must_stop_at_end_(must_stop_at_end), period_increment_(period_increment),
          speed_mode_(speed_mode)
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

    /// Get period increment
    uint16_t period_increment() const
    {
        return period_increment_;
    }

    /// Set period increment
    void set_period_increment(uint16_t period_increment)
    {
        period_increment_ = period_increment;
    }

    /// Get speed mode flag
    bool speed_mode() const
    {
        return speed_mode_;
    }

    /// Set speed mode flag
    void set_speed_mode(bool speed_mode)
    {
        speed_mode_ = speed_mode;
    }

  private:
    float max_speed_;           ///< Maximum velocity limit
    float acceleration_;        ///< Maximum acceleration
    float deceleration_;        ///< Maximum deceleration
    bool must_stop_at_end_;     ///< Stop at end or continue
    uint16_t period_increment_; ///< Period increment per execute (for throttling)
    bool speed_mode_;           ///< Use target_speed + duration instead of pose_error
};

} // namespace motion_control

} // namespace cogip

/// @}
