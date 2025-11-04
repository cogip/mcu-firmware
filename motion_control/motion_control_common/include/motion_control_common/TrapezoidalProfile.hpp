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
 *     double velocity = profile.compute_theoretical_velocity(period);
 *     double remaining = profile.compute_theoretical_remaining_distance(period);
 * }
 * @endcode
 */
class TrapezoidalProfile
{
  public:
    /// Constructor
    TrapezoidalProfile();

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
    uint32_t generate_optimal_profile(double initial_velocity,
                                       double target_distance,
                                       double acceleration,
                                       double deceleration,
                                       double max_speed,
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
    double compute_theoretical_velocity(uint32_t period) const;

    /**
     * @brief Compute theoretical remaining distance at given period
     *
     * Returns how much distance should remain to the target at the specified
     * period according to the pre-computed profile.
     *
     * @param period Period index (0 = start of trajectory)
     * @return Remaining distance from this period to target (mm or rad)
     */
    double compute_theoretical_remaining_distance(uint32_t period) const;

    /**
     * @brief Get total number of periods for the trajectory
     * @return Total periods (acceleration + plateau + deceleration)
     */
    uint32_t total_periods() const
    {
        return accel_periods_ + plateau_periods_ + decel_periods_;
    }

    /**
     * @brief Get plateau velocity
     * @return Target velocity during constant velocity phase (mm/period or rad/period)
     */
    double plateau_velocity() const
    {
        return plateau_velocity_;
    }

    /**
     * @brief Get initial velocity
     * @return Starting velocity (mm/period or rad/period)
     */
    double initial_velocity() const
    {
        return initial_velocity_;
    }

    /**
     * @brief Get target distance
     * @return Total distance to travel (mm or rad)
     */
    double target_distance() const
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

  private:
    /**
     * @brief Compute distance traveled during acceleration phase
     * @param v0 Initial velocity
     * @param v1 Final velocity
     * @param accel Acceleration rate
     * @return Distance traveled
     */
    static double compute_accel_distance(double v0, double v1, double accel);

    /**
     * @brief Compute number of periods for acceleration/deceleration
     * @param v0 Initial velocity
     * @param v1 Final velocity
     * @param accel Acceleration rate
     * @return Number of periods
     */
    static uint32_t compute_accel_periods(double v0, double v1, double accel);

    double initial_velocity_;  ///< Starting velocity
    double plateau_velocity_;  ///< Plateau velocity (may be < max_speed)
    double target_distance_;   ///< Total distance to cover
    double final_velocity_;    ///< Final velocity (0 if must_stop_at_end)

    uint32_t accel_periods_;   ///< Number of periods for acceleration phase
    uint32_t plateau_periods_; ///< Number of periods for constant velocity phase
    uint32_t decel_periods_;   ///< Number of periods for deceleration phase

    double acceleration_;  ///< Acceleration rate used
    double deceleration_;  ///< Deceleration rate used

    bool initialized_; ///< Profile has been computed
};

} // namespace motion_control

} // namespace cogip

/// @}
