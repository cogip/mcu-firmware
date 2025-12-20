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

#define ENABLE_DEBUG 0
#include <debug.h>

#include <etl/algorithm.h>

namespace cogip {

namespace motion_control {

void TrapezoidalProfile::reset_to_stationary(float velocity)
{
    initial_velocity_ = velocity;
    plateau_velocity_ = velocity;
    target_distance_ = 0.0f;
    final_velocity_ = velocity;
    accel_periods_ = 0;
    plateau_periods_ = 0;
    decel_periods_ = 0;
    acceleration_ = 0.0f;
    deceleration_ = 0.0f;
    initialized_ = true;
}

float TrapezoidalProfile::compute_accel_distance(float v0, float v1, float accel)
{
    // Distance = |v1² - v0²| / (2 * a)
    // Handle division by zero
    if (etl::absolute(accel) < 1e-6f) {
        return 0.0f;
    }
    return etl::absolute((v1 * v1 - v0 * v0) / (2.0f * accel));
}

uint32_t TrapezoidalProfile::compute_accel_periods(float v0, float v1, float accel)
{
    // Number of periods = |v1 - v0| / a
    // Handle division by zero
    if (etl::absolute(accel) < 1e-6f) {
        return 0;
    }
    float periods = etl::absolute(v1 - v0) / accel;
    // Round to nearest integer: add 0.5 and truncate
    return static_cast<uint32_t>(periods + 0.5f);
}

float TrapezoidalProfile::compute_discrete_distance(float v0, float accel, uint32_t periods)
{
    // Kinematic equation: d = v0*t + 0.5*a*t²
    return (v0 * periods) + (0.5f * accel * periods * periods);
}

float TrapezoidalProfile::compute_triangular_peak_velocity(float distance, float v0, float vf,
                                                           float accel, float decel)
{
    // General formula for triangular profile peak velocity:
    // v_peak² = (2*a*d*distance + d*v0² + a*vf²) / (a + d)
    float v_peak_squared =
        (2.0f * accel * decel * distance + decel * v0 * v0 + accel * vf * vf) / (accel + decel);
    // Guard against floating-point precision issues.
    // Mathematically v_peak_squared is always positive (all input terms are positive),
    // but floating-point rounding errors on edge cases (very small distances, nearly
    // equal velocities) could produce slightly negative values. Return 0 to avoid NaN.
    if (v_peak_squared < 0.0f) {
        return 0.0f;
    }
    return std::sqrt(v_peak_squared);
}

uint32_t TrapezoidalProfile::generate_optimal_profile(float initial_velocity, float target_distance,
                                                      float acceleration, float deceleration,
                                                      float max_speed, bool must_stop_at_end)
{
    // Reset state
    initialized_ = false;

    // Validate inputs
    if (etl::absolute(target_distance) < 1e-6f) {
        // No distance to travel
        reset_to_stationary(initial_velocity);
        return 0;
    }

    if (acceleration <= 0.0f || deceleration <= 0.0f || max_speed <= 0.0f) {
        // Invalid parameters - reset to stationary state
        reset_to_stationary(initial_velocity);
        return 0;
    }

    // Store parameters
    initial_velocity_ = initial_velocity;
    target_distance_ = target_distance;
    acceleration_ = acceleration;
    deceleration_ = deceleration;
    final_velocity_ = must_stop_at_end ? 0.0f : max_speed;

    // Handle direction (negative distance means backward)
    float direction = (target_distance >= 0.0f) ? 1.0f : -1.0f;
    float abs_distance = etl::absolute(target_distance);
    float signed_max_speed = max_speed * direction;

    // Ensure initial velocity has correct sign
    float v0 = initial_velocity * direction;
    if (v0 < 0.0f) {
        v0 = 0.0f; // Cannot start with negative velocity in movement direction
    }

    // Calculate distances needed for acceleration and deceleration
    float accel_distance = compute_accel_distance(v0, signed_max_speed, acceleration);
    float decel_distance =
        compute_accel_distance(signed_max_speed, final_velocity_ * direction, deceleration);

    // Check if we can reach max_speed
    if ((accel_distance + decel_distance) <= abs_distance) {
        // Trapezoidal profile: can reach max_speed
        plateau_velocity_ = signed_max_speed;

        accel_periods_ = compute_accel_periods(v0, plateau_velocity_, acceleration);
        decel_periods_ =
            compute_accel_periods(plateau_velocity_, final_velocity_ * direction, deceleration);

        DEBUG("Trapezoidal profile: accel_dist=%.2f, decel_dist=%.2f, abs_dist=%.2f, "
              "plateau_vel=%.2f\n",
              accel_distance, decel_distance, abs_distance, plateau_velocity_);

        // Recalculate actual distances with discrete periods
        // Use absolute values (v0 >= 0, max_speed > 0) to ensure positive distances
        float actual_accel_distance = compute_discrete_distance(v0, acceleration_, accel_periods_);
        float actual_decel_distance =
            compute_discrete_distance(max_speed, -deceleration_, decel_periods_);

        // Remaining distance is covered at constant velocity
        float plateau_distance = abs_distance - actual_accel_distance - actual_decel_distance;
        // Guard against division by zero (should not happen since max_speed > 0)
        float abs_plateau_velocity = etl::absolute(plateau_velocity_);
        plateau_periods_ =
            (abs_plateau_velocity != 0.0f)
                ? static_cast<uint32_t>(etl::max(0.0f, plateau_distance / abs_plateau_velocity))
                : 0;
    } else {
        // Triangular profile: cannot reach max_speed
        float v_peak = compute_triangular_peak_velocity(abs_distance, v0, final_velocity_,
                                                        acceleration_, deceleration_) *
                       direction;

        plateau_velocity_ = v_peak;
        plateau_periods_ = 0; // No plateau in triangular profile

        accel_periods_ = compute_accel_periods(v0, plateau_velocity_, acceleration);
        decel_periods_ =
            compute_accel_periods(plateau_velocity_, final_velocity_ * direction, deceleration);

        DEBUG("Triangular profile: v_peak=%.2f, accel=%.4f, decel=%.4f, accel_periods=%lu, "
              "decel_periods=%lu\n",
              v_peak, acceleration_, deceleration_, (unsigned long)accel_periods_,
              (unsigned long)decel_periods_);
    }

    initialized_ = true;
    return total_periods();
}

float TrapezoidalProfile::compute_theoretical_velocity(uint32_t period) const
{
    if (!initialized_) {
        return 0.0f;
    }

    float velocity = 0.0f;
    const char* phase = "unknown";

    // Direction is determined by plateau_velocity sign
    float direction = (plateau_velocity_ >= 0.0f) ? 1.0f : -1.0f;

    if (period < accel_periods_) {
        // Acceleration phase: v(t) = v0 + direction*a*t
        velocity = initial_velocity_ + (direction * acceleration_ * period);
        phase = "accel";
    } else if (period < (accel_periods_ + plateau_periods_)) {
        // Plateau phase: constant velocity
        velocity = plateau_velocity_;
        phase = "plateau";
    } else if (period < total_periods()) {
        // Deceleration phase: v(t) = v_plateau - direction*d*(t - t_plateau)
        uint32_t decel_period = period - accel_periods_ - plateau_periods_;
        velocity = plateau_velocity_ - (direction * deceleration_ * decel_period);
        phase = "decel";
    } else {
        // After trajectory completion
        velocity = final_velocity_;
        phase = "done";
    }

    DEBUG("TrapezoidalProfile: period=%lu, phase=%s, velocity=%.2f, accel_periods=%lu, "
          "plateau_periods=%lu, decel_periods=%lu\n",
          (unsigned long)period, phase, velocity, (unsigned long)accel_periods_,
          (unsigned long)plateau_periods_, (unsigned long)decel_periods_);

    return velocity;
}

float TrapezoidalProfile::compute_theoretical_remaining_distance(uint32_t period) const
{
    if (!initialized_) {
        return 0.0f;
    }

    // Direction is determined by plateau_velocity sign
    float direction = (plateau_velocity_ >= 0.0f) ? 1.0f : -1.0f;

    // Calculate distance already traveled up to this period
    float distance_traveled = 0.0f;

    // Acceleration phase contribution
    if (period > 0) {
        uint32_t accel_contribution = etl::min(period, accel_periods_);
        distance_traveled +=
            initial_velocity_ * accel_contribution +
            direction * 0.5f * acceleration_ * accel_contribution * accel_contribution;
    }

    // Plateau phase contribution
    if (period > accel_periods_) {
        uint32_t plateau_contribution = etl::min(period - accel_periods_, plateau_periods_);
        distance_traveled += plateau_velocity_ * plateau_contribution;
    }

    // Deceleration phase contribution
    if (period > (accel_periods_ + plateau_periods_)) {
        uint32_t decel_contribution =
            etl::min(period - accel_periods_ - plateau_periods_, decel_periods_);
        distance_traveled +=
            plateau_velocity_ * decel_contribution -
            direction * 0.5f * deceleration_ * decel_contribution * decel_contribution;
    }

    // Remaining distance = target - traveled
    return target_distance_ - distance_traveled;
}

} // namespace motion_control

} // namespace cogip

/// @}
