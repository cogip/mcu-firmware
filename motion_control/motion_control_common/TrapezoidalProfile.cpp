// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Trapezoidal velocity profile implementation
/// @author     Gilles Doffe <g.doffe@gmail.com>

#include "motion_control_common/TrapezoidalProfile.hpp"

#include <cmath>

#include <etl/algorithm.h>

namespace cogip {

namespace motion_control {

TrapezoidalProfile::TrapezoidalProfile()
    : initial_velocity_(0.0),
      plateau_velocity_(0.0),
      target_distance_(0.0),
      final_velocity_(0.0),
      accel_periods_(0),
      plateau_periods_(0),
      decel_periods_(0),
      acceleration_(0.0),
      deceleration_(0.0),
      initialized_(false)
{
}

double TrapezoidalProfile::compute_accel_distance(double v0, double v1, double accel)
{
    // Distance = (v1² - v0²) / (2 * a)
    // Handle division by zero
    if (etl::absolute(accel) < 1e-6) {
        return 0.0;
    }
    return (v1 * v1 - v0 * v0) / (2.0 * accel);
}

uint32_t TrapezoidalProfile::compute_accel_periods(double v0, double v1, double accel)
{
    // Number of periods = (v1 - v0) / a
    // Handle division by zero
    if (etl::absolute(accel) < 1e-6) {
        return 0;
    }
    double periods = (v1 - v0) / accel;
    // Round to nearest integer: add 0.5 and truncate
    return static_cast<uint32_t>(etl::max(0.0, periods + 0.5));
}

uint32_t TrapezoidalProfile::generate_optimal_profile(double initial_velocity,
                                                       double target_distance,
                                                       double acceleration,
                                                       double deceleration,
                                                       double max_speed,
                                                       bool must_stop_at_end)
{
    // Reset state
    initialized_ = false;

    // Validate inputs
    if (etl::absolute(target_distance) < 1e-6) {
        // No distance to travel
        initial_velocity_ = initial_velocity;
        plateau_velocity_ = initial_velocity;
        target_distance_ = 0.0;
        final_velocity_ = initial_velocity;
        accel_periods_ = 0;
        plateau_periods_ = 0;
        decel_periods_ = 0;
        acceleration_ = 0.0;
        deceleration_ = 0.0;
        initialized_ = true;
        return 0;
    }

    if (acceleration <= 0.0 || deceleration <= 0.0 || max_speed <= 0.0) {
        return 0; // Invalid parameters
    }

    // Store parameters
    initial_velocity_ = initial_velocity;
    target_distance_ = target_distance;
    acceleration_ = acceleration;
    deceleration_ = deceleration;
    final_velocity_ = must_stop_at_end ? 0.0 : max_speed;

    // Handle direction (negative distance means backward)
    double direction = (target_distance >= 0.0) ? 1.0 : -1.0;
    double abs_distance = etl::absolute(target_distance);
    double signed_max_speed = max_speed * direction;

    // Ensure initial velocity has correct sign
    double v0 = initial_velocity * direction;
    if (v0 < 0.0) {
        v0 = 0.0; // Cannot start with negative velocity in movement direction
    }

    // Calculate distances needed for acceleration and deceleration
    double accel_distance = compute_accel_distance(v0, signed_max_speed, acceleration);
    double decel_distance = compute_accel_distance(signed_max_speed, final_velocity_ * direction, deceleration);

    // Check if we can reach max_speed
    if ((accel_distance + decel_distance) <= abs_distance) {
        // Trapezoidal profile: can reach max_speed
        plateau_velocity_ = signed_max_speed;

        accel_periods_ = compute_accel_periods(v0, plateau_velocity_, acceleration);
        decel_periods_ = compute_accel_periods(plateau_velocity_, final_velocity_ * direction, deceleration);

        // Recalculate actual distances with discrete periods
        double actual_accel_distance =
            (v0 * accel_periods_) + (0.5 * acceleration_ * accel_periods_ * accel_periods_);
        double actual_decel_distance = (plateau_velocity_ * decel_periods_) -
                                       (0.5 * deceleration_ * decel_periods_ * decel_periods_);

        // Remaining distance is covered at constant velocity
        double plateau_distance = abs_distance - actual_accel_distance - actual_decel_distance;
        plateau_periods_ = static_cast<uint32_t>(etl::max(0.0, plateau_distance / etl::absolute(plateau_velocity_)));
    }
    else {
        // Triangular profile: cannot reach max_speed
        // Find peak velocity where accel_distance + decel_distance = target_distance
        // v_peak² / (2*a) + v_peak² / (2*d) = distance
        // v_peak² * (1/(2*a) + 1/(2*d)) = distance
        // v_peak = sqrt(distance * 2 * a * d / (a + d))

        double v_peak_squared = abs_distance * 2.0 * acceleration_ * deceleration_ /
                                (acceleration_ + deceleration_);
        double v_peak = std::sqrt(v_peak_squared) * direction;

        plateau_velocity_ = v_peak;
        plateau_periods_ = 0; // No plateau in triangular profile

        accel_periods_ = compute_accel_periods(v0, plateau_velocity_, acceleration);
        decel_periods_ = compute_accel_periods(plateau_velocity_, final_velocity_ * direction, deceleration);
    }

    initialized_ = true;
    return total_periods();
}

double TrapezoidalProfile::compute_theoretical_velocity(uint32_t period) const
{
    if (!initialized_) {
        return 0.0;
    }

    if (period < accel_periods_) {
        // Acceleration phase: v(t) = v0 + a*t
        return initial_velocity_ + (acceleration_ * period);
    }
    else if (period < (accel_periods_ + plateau_periods_)) {
        // Plateau phase: constant velocity
        return plateau_velocity_;
    }
    else if (period < total_periods()) {
        // Deceleration phase: v(t) = v_plateau - d*(t - t_plateau)
        uint32_t decel_period = period - accel_periods_ - plateau_periods_;
        return plateau_velocity_ - (deceleration_ * decel_period);
    }
    else {
        // After trajectory completion
        return final_velocity_;
    }
}

double TrapezoidalProfile::compute_theoretical_remaining_distance(uint32_t period) const
{
    if (!initialized_) {
        return 0.0;
    }

    // Calculate distance already traveled up to this period
    double distance_traveled = 0.0;

    // Acceleration phase contribution
    if (period > 0) {
        uint32_t accel_contribution = etl::min(period, accel_periods_);
        distance_traveled += initial_velocity_ * accel_contribution +
                             0.5 * acceleration_ * accel_contribution * accel_contribution;
    }

    // Plateau phase contribution
    if (period > accel_periods_) {
        uint32_t plateau_contribution = etl::min(period - accel_periods_, plateau_periods_);
        distance_traveled += plateau_velocity_ * plateau_contribution;
    }

    // Deceleration phase contribution
    if (period > (accel_periods_ + plateau_periods_)) {
        uint32_t decel_contribution = etl::min(period - accel_periods_ - plateau_periods_, decel_periods_);
        distance_traveled += plateau_velocity_ * decel_contribution -
                             0.5 * deceleration_ * decel_contribution * decel_contribution;
    }

    // Remaining distance = target - traveled
    return target_distance_ - distance_traveled;
}

} // namespace motion_control

} // namespace cogip

/// @}
