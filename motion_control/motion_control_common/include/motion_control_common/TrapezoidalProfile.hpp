// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Trapezoidal velocity profile calculator
/// @author     Gilles Doffe <g.doffe@gmail.com>

#pragma once

#include <cstdint>

namespace cogip {

namespace motion_control {

/**
 * @brief Trapezoidal velocity profile calculator with automatic optimal timing
 *
 * This class generates an optimal trapezoidal (or triangular) velocity profile
 * to reach a target distance as fast as possible while respecting velocity,
 * acceleration and deceleration constraints.
 *
 * Two profile types can be generated:
 * - Trapezoidal: acceleration phase, constant velocity plateau, deceleration phase
 * - Triangular: acceleration phase directly followed by deceleration (no plateau)
 *
 * The profile is computed in discrete time periods (control loop periods).
 *
 * Usage example:
 * @code
 * TrapezoidalProfile profile;
 * uint32_t total_periods = profile.generate_optimal_profile(
 *     0.0,      // initial_velocity
 *     1000.0,   // target_distance (mm)
 *     10.0,     // acceleration (mm/period²)
 *     10.0,     // deceleration (mm/period²)
 *     50.0,     // max_speed (mm/period)
 *     true      // must_stop_at_end
 * );
 *
 * for (uint32_t period = 0; period < total_periods; period++) {
 *     float velocity = profile.compute_theoretical_velocity(period);
 *     float remaining = profile.compute_theoretical_remaining_distance(period);
 * }
 * @endcode
 */
class TrapezoidalProfile
{
  public:
    /// Default constructor
    TrapezoidalProfile() = default;

    /**
     * @brief Generate optimal velocity profile to reach target distance
     *
     * Computes the fastest possible trajectory to reach target_distance while
     * respecting all physical constraints (max speed, acceleration, deceleration).
     *
     * The algorithm automatically determines:
     * - Whether to generate a trapezoidal or triangular profile
     * - The plateau velocity (may be less than max_speed for short distances)
     * - The number of periods for each phase (acceleration, plateau, deceleration)
     *
     * @param initial_velocity Starting velocity (mm/period or rad/period)
     * @param target_distance Distance to travel (mm or rad, can be negative)
     * @param acceleration Maximum acceleration (mm/period² or rad/period²)
     * @param deceleration Maximum deceleration (mm/period² or rad/period²)
     * @param max_speed Maximum velocity limit (mm/period or rad/period)
     * @param must_stop_at_end If true, final velocity is 0; if false, continues at plateau velocity
     * @return Total number of periods to complete the trajectory, 0 on error
     */
    uint32_t generate_optimal_profile(float initial_velocity, float target_distance,
                                      float acceleration, float deceleration, float max_speed,
                                      bool must_stop_at_end);

    /**
     * @brief Compute theoretical velocity at given period
     *
     * Returns the velocity the system should have at the specified period
     * according to the pre-computed profile.
     *
     * @param period Period index (0 = start of trajectory)
     * @return Theoretical velocity at this period (mm/period or rad/period)
     */
    float compute_theoretical_velocity(uint32_t period) const;

    /**
     * @brief Compute theoretical remaining distance at given period
     *
     * Returns how much distance should remain to the target at the specified
     * period according to the pre-computed profile.
     *
     * @param period Period index (0 = start of trajectory)
     * @return Remaining distance from this period to target (mm or rad)
     */
    float compute_theoretical_remaining_distance(uint32_t period) const;

    /**
     * @brief Get total number of periods for the trajectory
     * @return Total periods (reverse_decel + acceleration + plateau + deceleration)
     */
    uint32_t total_periods() const
    {
        return reverse_decel_periods_ + accel_periods_ + plateau_periods_ + decel_periods_;
    }

    /**
     * @brief Get plateau velocity
     * @return Target velocity during constant velocity phase (mm/period or rad/period)
     */
    float plateau_velocity() const
    {
        return plateau_velocity_;
    }

    /**
     * @brief Get initial velocity
     * @return Starting velocity (mm/period or rad/period)
     */
    float initial_velocity() const
    {
        return initial_velocity_;
    }

    /**
     * @brief Get target distance
     * @return Total distance to travel (mm or rad)
     */
    float target_distance() const
    {
        return target_distance_;
    }

    /**
     * @brief Check if profile is initialized
     * @return true if generate_optimal_profile() was called successfully
     */
    bool is_initialized() const
    {
        return initialized_;
    }

    /**
     * @brief Reset profile to uninitialized state
     *
     * After reset, all compute methods return 0 until a new profile is generated.
     * This allows the controller to rely solely on the profile state without
     * maintaining a separate ready flag.
     */
    void reset()
    {
        initialized_ = false;
    }

  private:
    /**
     * @brief Reset profile to stationary state (no movement)
     * @param velocity Current velocity (used as initial/final/plateau velocity)
     */
    void reset_to_stationary(float velocity);

    /**
     * @brief Compute absolute distance traveled during acceleration phase
     * @param v0 Initial velocity
     * @param v1 Final velocity
     * @param accel Acceleration rate
     * @return Absolute distance traveled: |v1² - v0²| / (2*a)
     */
    static float compute_accel_distance(float v0, float v1, float accel);

    /**
     * @brief Compute number of periods for acceleration/deceleration
     * @param v0 Initial velocity
     * @param v1 Final velocity
     * @param accel Acceleration rate
     * @return Number of periods
     */
    static uint32_t compute_accel_periods(float v0, float v1, float accel);

    /**
     * @brief Compute distance using kinematic equation with discrete periods
     * @param v0 Initial velocity
     * @param accel Acceleration (positive for accel, negative for decel)
     * @param periods Number of discrete periods
     * @return Distance traveled: d = v0*t + 0.5*a*t²
     */
    static float compute_discrete_distance(float v0, float accel, uint32_t periods);

    /**
     * @brief Compute peak velocity for triangular profile
     * @param distance Total distance to cover
     * @param v0 Initial velocity
     * @param vf Final velocity
     * @param accel Acceleration rate
     * @param decel Deceleration rate
     * @return Peak velocity: v_peak² = (2*a*d*dist + d*v0² + a*vf²) / (a+d)
     */
    static float compute_triangular_peak_velocity(float distance, float v0, float vf, float accel,
                                                  float decel);

    float initial_velocity_ = 0.0f; ///< Starting velocity (signed, can be opposite to target)
    float v0_for_accel_ =
        0.0f; ///< Velocity at start of accel phase (0 if reverse_decel, else projected initial)
    float plateau_velocity_ = 0.0f; ///< Plateau velocity (may be < max_speed)
    float target_distance_ = 0.0f;  ///< Total distance to cover
    float final_velocity_ = 0.0f;   ///< Final velocity (0 if must_stop_at_end)

    uint32_t reverse_decel_periods_ = 0; ///< Periods to decel from opposite direction to 0
    uint32_t accel_periods_ = 0;         ///< Number of periods for acceleration phase
    uint32_t plateau_periods_ = 0;       ///< Number of periods for constant velocity phase
    uint32_t decel_periods_ = 0;         ///< Number of periods for deceleration phase

    float acceleration_ = 0.0f; ///< Acceleration rate used
    float deceleration_ = 0.0f; ///< Deceleration rate used
    float initial_phase_accel_ =
        0.0f; ///< Signed accel for first phase (+a if speeding up, -d if slowing down)

    bool initialized_ = false; ///< Profile has been computed
};

} // namespace motion_control

} // namespace cogip

/// @}
