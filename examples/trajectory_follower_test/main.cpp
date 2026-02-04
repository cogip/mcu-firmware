// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/**
 * @brief Test application for TrajectoryFollowerController
 *
 * This example demonstrates a trajectory follower with tracker + feedback control.
 * It simulates a robot moving from 0 to 1000mm with:
 * - Tracker: pre-computed trapezoidal velocity profile
 * - Feedback: PID correction for position tracking errors
 */

#include <cstdio>
#include <iostream>

#include "motion_control_common/ControllersIO.hpp"
#include "motion_control_common/TrapezoidalProfile.hpp"

using cogip::motion_control::ControllersIO;
using cogip::motion_control::TrapezoidalProfile;

namespace {

// Simple PID class for testing (no protobuf dependencies)
class SimplePID
{
  public:
    SimplePID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0)
    {
    }

    double compute(double error)
    {
        integral_ += error;
        double derivative = error - previous_error_;
        previous_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

  private:
    double kp_, ki_, kd_;
    double integral_;
    double previous_error_;
};

// Simple robot simulator
class RobotSimulator
{
  public:
    RobotSimulator() : position_(0.0), velocity_(0.0) {}

    void update(double velocity_command)
    {
        // Simple integration: position += velocity * dt
        // (dt = 1 period in our case)
        velocity_ = velocity_command;
        position_ += velocity_;
    }

    double position() const
    {
        return position_;
    }
    double velocity() const
    {
        return velocity_;
    }

  private:
    double position_;
    double velocity_;
};

} // namespace

/**
 * @brief Run a single trajectory follower test
 * @param test_name Name of the test
 * @param target_distance Distance to travel (can be negative for backward)
 * @param max_speed Maximum velocity
 * @param acceleration Acceleration rate
 * @param deceleration Deceleration rate
 * @return true if test passed
 */
bool run_test(const char* test_name, double target_distance, double max_speed, double acceleration,
              double deceleration)
{
    std::cout << "\n-------------------------------------------\n";
    std::cout << "Test: " << test_name << "\n";
    std::cout << "-------------------------------------------\n";

    // Create PID for position error correction
    SimplePID pid(0.1, 0.0, 0.0); // Simple P controller

    // Create trapezoidal profile
    TrapezoidalProfile profile;

    // Create robot simulator
    RobotSimulator robot;

    uint32_t total_periods =
        profile.generate_optimal_profile(robot.velocity(), // initial velocity
                                         target_distance, acceleration, deceleration, max_speed,
                                         true // must_stop_at_end
        );

    std::cout << "Target: " << target_distance << " mm\n";
    std::cout << "Max speed: " << max_speed << " mm/period\n";
    std::cout << "Acceleration: " << acceleration << " mm/period²\n";
    std::cout << "Deceleration: " << deceleration << " mm/period²\n";
    std::cout << "Expected duration: " << total_periods << " periods\n\n";

    std::cout << "Period | Position (mm) | Velocity (mm/period) | FF Velocity | FB Correction | "
                 "Distance Remaining\n";
    std::cout << "-------|---------------|----------------------|-------------|---------------|----"
                 "---------------\n";

    // Simulation loop with tracker + feedback
    double distance_threshold = 1.0; // mm
    bool target_reached = false;
    for (uint32_t period = 0; period <= total_periods + 10; period++) {
        // Compute actual remaining distance
        double actual_remaining = target_distance - robot.position();

        // Check if target reached
        if (std::abs(actual_remaining) < distance_threshold) {
            printf("%6u | %13.2f | %20.2f |      -      |       -       | %17.2f\n", period,
                   robot.position(), robot.velocity(), actual_remaining);
            std::cout << "\nTarget reached at period " << period << "!\n";
            std::cout << "Final position: " << robot.position() << " mm\n";
            std::cout << "Final velocity: " << robot.velocity() << " mm/period\n";
            target_reached = true;
            break;
        }

        // Compute tracker velocity from profile
        double tracker_velocity = profile.compute_theoretical_velocity(period);

        // Compute theoretical remaining distance from profile
        double theoretical_remaining = profile.compute_theoretical_remaining_distance(period);

        // Compute position error: actual - theoretical
        double position_error = actual_remaining - theoretical_remaining;

        // Compute feedback correction via PID
        double feedback_correction = pid.compute(position_error);

        // Final velocity command: tracker + feedback
        double velocity_command = tracker_velocity + feedback_correction;

        // Update robot simulator
        robot.update(velocity_command);

        // Print every 10 periods (or every period for short profiles)
        if (period % 10 == 0 || total_periods <= 20) {
            printf("%6u | %13.2f | %20.2f | %11.2f | %13.2f | %17.2f\n", period, robot.position(),
                   robot.velocity(), tracker_velocity, feedback_correction, actual_remaining);
        }
    }

    return target_reached;
}

int main(void)
{
    std::cout << "===========================================\n";
    std::cout << "   TrajectoryFollowerController Test\n";
    std::cout << "===========================================\n";

    int passed = 0;
    int failed = 0;

    // Test 1: Long distance forward (trapezoidal profile)
    if (run_test("Long distance forward (trapezoidal)", 1000.0, 10.0, 1.0, 1.0)) {
        passed++;
    } else {
        failed++;
    }

    // Test 2: Short distance forward (triangular profile)
    if (run_test("Short distance forward (triangular)", 50.0, 10.0, 1.0, 1.0)) {
        passed++;
    } else {
        failed++;
    }

    // Test 3: Long distance backward (trapezoidal profile)
    if (run_test("Long distance backward (trapezoidal)", -1000.0, 10.0, 1.0, 1.0)) {
        passed++;
    } else {
        failed++;
    }

    // Test 4: Short distance backward (triangular profile)
    if (run_test("Short distance backward (triangular)", -50.0, 10.0, 1.0, 1.0)) {
        passed++;
    } else {
        failed++;
    }

    std::cout << "\n===========================================\n";
    std::cout << "   Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "===========================================\n";

    return failed > 0 ? 1 : 0;
}
