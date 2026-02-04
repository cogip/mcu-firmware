// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/**
 * @brief Test application for TrapezoidalProfile class
 *
 * This example demonstrates the usage of TrapezoidalProfile to generate
 * optimal velocity profiles for different scenarios:
 * 1. Long distance (trapezoidal profile with plateau)
 * 2. Short distance (triangular profile without plateau)
 * 3. Starting with non-zero velocity
 * 4. Backward movement (negative distance)
 */

#include <cstdio>
#include <inttypes.h>
#include <iostream>

#include "etl/algorithm.h"

#include "motion_control_common/TrapezoidalProfile.hpp"

using cogip::motion_control::TrapezoidalProfile;

/**
 * @brief Print profile summary and detailed trajectory
 */
void print_profile(const TrapezoidalProfile& profile, const char* test_name)
{
    std::cout << "\n========================================\n";
    std::cout << "Test: " << test_name << "\n";
    std::cout << "========================================\n";

    std::cout << "Initial velocity: " << profile.initial_velocity() << " mm/period\n";
    std::cout << "Target distance:  " << profile.target_distance() << " mm\n";
    std::cout << "Plateau velocity: " << profile.plateau_velocity() << " mm/period\n";
    std::cout << "Total periods:    " << profile.total_periods() << "\n";
    std::cout << "Total time:       " << (profile.total_periods() * 20) << " ms (at 20ms/period)\n";

    std::cout << "\nPeriod | Velocity (mm/period) | Remaining Distance (mm)\n";
    std::cout << "-------|----------------------|------------------------\n";

    uint32_t total = profile.total_periods();
    // Print first 10 periods
    for (uint32_t period = 0; period <= etl::min(static_cast<uint32_t>(10), total); period++) {
        double velocity = profile.compute_theoretical_velocity(period);
        double remaining = profile.compute_theoretical_remaining_distance(period);
        printf("%6" PRIu32 " | %20.2f | %22.2f\n", period, velocity, remaining);
    }

    // Print around middle
    if (total > 20) {
        std::cout << "  ...  |         ...          |          ...\n";
        uint32_t mid = total / 2;
        for (uint32_t period = mid - 2; period <= mid + 2; period++) {
            double velocity = profile.compute_theoretical_velocity(period);
            double remaining = profile.compute_theoretical_remaining_distance(period);
            printf("%6" PRIu32 " | %20.2f | %22.2f\n", period, velocity, remaining);
        }
    }

    // Print last 10 periods
    if (total > 10) {
        std::cout << "  ...  |         ...          |          ...\n";
        for (uint32_t period = total - 5; period <= total; period++) {
            double velocity = profile.compute_theoretical_velocity(period);
            double remaining = profile.compute_theoretical_remaining_distance(period);
            printf("%6" PRIu32 " | %20.2f | %22.2f\n", period, velocity, remaining);
        }
    }

    // Verify final remaining distance is ~0
    double final_remaining =
        profile.compute_theoretical_remaining_distance(profile.total_periods());
    std::cout << "\nFinal remaining distance: " << final_remaining << " mm\n";
    if (std::abs(final_remaining) < 1.0) {
        std::cout << "✓ Profile is consistent (remaining ≈ 0)\n";
    } else {
        std::cout << "✗ WARNING: Profile inconsistent (remaining should be ≈ 0)\n";
    }
}

/**
 * @brief Test 1: Long distance - Trapezoidal profile
 */
void test_long_distance()
{
    TrapezoidalProfile profile;

    // Simulate robot parameters (per 20ms period)
    double initial_velocity = 0.0;   // Start from rest (mm/period)
    double target_distance = 1000.0; // 1 meter
    double acceleration = 1.0;       // 1 mm/period² = 2500 mm/s²
    double deceleration = 1.0;       // 1 mm/period²
    double max_speed = 10.0;         // 10 mm/period = 500 mm/s
    bool must_stop_at_end = true;

    profile.generate_optimal_profile(initial_velocity, target_distance, acceleration, deceleration,
                                     max_speed, must_stop_at_end);

    print_profile(profile, "Long Distance (Trapezoidal)");
}

/**
 * @brief Test 2: Short distance - Triangular profile
 */
void test_short_distance()
{
    TrapezoidalProfile profile;

    // Short distance: won't reach max_speed
    double initial_velocity = 0.0;
    double target_distance = 50.0; // 50 mm (too short for plateau)
    double acceleration = 1.0;
    double deceleration = 1.0;
    double max_speed = 10.0;
    bool must_stop_at_end = true;

    profile.generate_optimal_profile(initial_velocity, target_distance, acceleration, deceleration,
                                     max_speed, must_stop_at_end);

    print_profile(profile, "Short Distance (Triangular)");
}

/**
 * @brief Test 3: Starting with non-zero velocity
 */
void test_with_initial_velocity()
{
    TrapezoidalProfile profile;

    double initial_velocity = 5.0; // Already moving at 5 mm/period
    double target_distance = 500.0;
    double acceleration = 0.5;
    double deceleration = 0.5;
    double max_speed = 10.0;
    bool must_stop_at_end = true;

    profile.generate_optimal_profile(initial_velocity, target_distance, acceleration, deceleration,
                                     max_speed, must_stop_at_end);

    print_profile(profile, "With Initial Velocity");
}

/**
 * @brief Test 4: Backward movement
 */
void test_backward_movement()
{
    TrapezoidalProfile profile;

    double initial_velocity = 0.0;
    double target_distance = -300.0; // Negative distance = backward
    double acceleration = 1.0;
    double deceleration = 1.0;
    double max_speed = 8.0;
    bool must_stop_at_end = true;

    profile.generate_optimal_profile(initial_velocity, target_distance, acceleration, deceleration,
                                     max_speed, must_stop_at_end);

    print_profile(profile, "Backward Movement");
}

/**
 * @brief Test 5: Continue at speed (no final stop)
 */
void test_continue_at_speed()
{
    TrapezoidalProfile profile;

    double initial_velocity = 0.0;
    double target_distance = 500.0;
    double acceleration = 1.0;
    double deceleration = 1.0;
    double max_speed = 10.0;
    bool must_stop_at_end = false; // Continue at max_speed

    profile.generate_optimal_profile(initial_velocity, target_distance, acceleration, deceleration,
                                     max_speed, must_stop_at_end);

    print_profile(profile, "Continue at Speed (Intermediate Waypoint)");
}

int main(void)
{
    std::cout << "===========================================\n";
    std::cout << "   TrapezoidalProfile Test Application\n";
    std::cout << "===========================================\n";
    std::cout << "\nAll units in mm and periods (1 period = 20ms)\n";

    test_long_distance();
    test_short_distance();
    test_with_initial_velocity();
    test_backward_movement();
    test_continue_at_speed();

    std::cout << "\n===========================================\n";
    std::cout << "   All tests completed!\n";
    std::cout << "===========================================\n";

    return 0;
}
