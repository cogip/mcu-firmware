// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    adaptive_pure_pursuit_controller
/// @{
/// @file
/// @brief      Adaptive Pure Pursuit controller parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Parameters for AdaptivePurePursuitController.
class AdaptivePurePursuitControllerParameters
{
  public:
    /// @brief Constructor with all parameters.
    /// @param min_lookahead_distance       Minimum lookahead distance (mm)
    /// @param max_lookahead_distance       Maximum lookahead distance (mm)
    /// @param lookahead_speed_ratio        Ratio to adapt lookahead to speed (mm / (mm/period))
    /// @param max_linear_speed             Maximum linear speed (mm/period)
    /// @param max_angular_speed            Maximum angular speed (deg/period)
    /// @param linear_threshold             Tolerance to consider path complete (mm)
    /// @param angular_threshold            Tolerance for final orientation (deg)
    /// @param initial_rotation_threshold   Tolerance for ROTATING_TO_DIRECTION state (deg)
    /// @param linear_acceleration          Linear acceleration (mm/period²)
    /// @param linear_deceleration          Linear deceleration (mm/period²)
    /// @param angular_deceleration         Angular deceleration (deg/period²)
    explicit AdaptivePurePursuitControllerParameters(
        float min_lookahead_distance = 100.0f, float max_lookahead_distance = 300.0f,
        float lookahead_speed_ratio = 10.0f, float max_linear_speed = 10.0f,
        float max_angular_speed = 5.0f, float linear_threshold = 10.0f,
        float angular_threshold = 2.0f, float initial_rotation_threshold = 45.0f,
        float linear_acceleration = 0.1f, float linear_deceleration = 0.1f,
        float angular_deceleration = 0.1f)
        : min_lookahead_distance_(min_lookahead_distance),
          max_lookahead_distance_(max_lookahead_distance),
          lookahead_speed_ratio_(lookahead_speed_ratio), max_linear_speed_(max_linear_speed),
          max_angular_speed_(max_angular_speed), linear_threshold_(linear_threshold),
          angular_threshold_(angular_threshold),
          initial_rotation_threshold_(initial_rotation_threshold),
          linear_acceleration_(linear_acceleration), linear_deceleration_(linear_deceleration),
          angular_deceleration_(angular_deceleration)
    {
    }

    /// @brief Get minimum lookahead distance.
    float min_lookahead_distance() const
    {
        return min_lookahead_distance_;
    }

    /// @brief Get maximum lookahead distance.
    float max_lookahead_distance() const
    {
        return max_lookahead_distance_;
    }

    /// @brief Get lookahead speed ratio.
    float lookahead_speed_ratio() const
    {
        return lookahead_speed_ratio_;
    }

    /// @brief Get maximum linear speed.
    float max_linear_speed() const
    {
        return max_linear_speed_;
    }

    /// @brief Get maximum angular speed.
    float max_angular_speed() const
    {
        return max_angular_speed_;
    }

    /// @brief Get linear threshold.
    float linear_threshold() const
    {
        return linear_threshold_;
    }

    /// @brief Get angular tolerance.
    float angular_threshold() const
    {
        return angular_threshold_;
    }

    /// @brief Get initial rotation tolerance.
    float initial_rotation_threshold() const
    {
        return initial_rotation_threshold_;
    }

    /// @brief Set minimum lookahead distance.
    void set_min_lookahead_distance(float value)
    {
        min_lookahead_distance_ = value;
    }

    /// @brief Set maximum lookahead distance.
    void set_max_lookahead_distance(float value)
    {
        max_lookahead_distance_ = value;
    }

    /// @brief Set lookahead speed ratio.
    void set_lookahead_speed_ratio(float value)
    {
        lookahead_speed_ratio_ = value;
    }

    /// @brief Set maximum linear speed.
    void set_max_linear_speed(float value)
    {
        max_linear_speed_ = value;
    }

    /// @brief Set maximum angular speed.
    void set_max_angular_speed(float value)
    {
        max_angular_speed_ = value;
    }

    /// @brief Set linear threshold.
    void set_linear_threshold(float value)
    {
        linear_threshold_ = value;
    }

    /// @brief Set angular tolerance.
    void set_angular_threshold(float value)
    {
        angular_threshold_ = value;
    }

    /// @brief Set initial rotation tolerance.
    void set_initial_rotation_threshold(float value)
    {
        initial_rotation_threshold_ = value;
    }

    /// @brief Get linear acceleration.
    float linear_acceleration() const
    {
        return linear_acceleration_;
    }

    /// @brief Set linear acceleration.
    void set_linear_acceleration(float value)
    {
        linear_acceleration_ = value;
    }

    /// @brief Get linear deceleration.
    float linear_deceleration() const
    {
        return linear_deceleration_;
    }

    /// @brief Set linear deceleration.
    void set_linear_deceleration(float value)
    {
        linear_deceleration_ = value;
    }

    /// @brief Get angular deceleration.
    float angular_deceleration() const
    {
        return angular_deceleration_;
    }

    /// @brief Set angular deceleration.
    void set_angular_deceleration(float value)
    {
        angular_deceleration_ = value;
    }

  private:
    float min_lookahead_distance_;     ///< Minimum lookahead distance (mm)
    float max_lookahead_distance_;     ///< Maximum lookahead distance (mm)
    float lookahead_speed_ratio_;      ///< Ratio to adapt lookahead to speed
    float max_linear_speed_;           ///< Maximum linear speed (mm/period)
    float max_angular_speed_;          ///< Maximum angular speed (deg/period)
    float linear_threshold_;           ///< Tolerance to consider path complete (mm)
    float angular_threshold_;          ///< Tolerance for final orientation (deg)
    float initial_rotation_threshold_; ///< Tolerance for ROTATING_TO_DIRECTION state (deg)
    float linear_acceleration_;        ///< Linear acceleration (mm/period²)
    float linear_deceleration_;        ///< Linear deceleration (mm/period²)
    float angular_deceleration_;       ///< Angular deceleration (deg/period²)
};

} // namespace motion_control

} // namespace cogip

/// @}
