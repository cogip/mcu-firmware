// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/**
 * @brief Test application for Tracker Chain
 *
 * This example demonstrates the complete tracker + feedback control chain:
 *
 * ProfileTrackerController → generates tracker + tracking_error
 *             ↓
 * PosePIDController → correction from tracking_error
 *             ↓
 * TrackerCombinerController → combines tracker + correction
 *
 * Simulates a robot moving from 0 to 1000mm with trapezoidal profile.
 */

#include <cstdio>
#include <iostream>

#include "motion_control_common/ControllersIO.hpp"
#include "profile_tracker_controller/ProfileTrackerController.hpp"
#include "tracker_combiner_controller/TrackerCombinerController.hpp"

using cogip::motion_control::ControllersIO;
using cogip::motion_control::ProfileTrackerController;
using cogip::motion_control::ProfileTrackerControllerIOKeys;
using cogip::motion_control::ProfileTrackerControllerParameters;
using cogip::motion_control::TrackerCombinerController;
using cogip::motion_control::TrackerCombinerControllerIOKeys;
using cogip::motion_control::TrackerCombinerControllerParameters;

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

    void reset()
    {
        integral_ = 0.0;
        previous_error_ = 0.0;
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
    RobotSimulator() : position_(0.0), velocity_(0.0), target_position_(0.0) {}

    void set_target(double target)
    {
        target_position_ = target;
    }

    void update(double velocity_command)
    {
        // Simple integration: position += velocity * dt (dt = 1 period)
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

    double remaining_distance() const
    {
        return target_position_ - position_;
    }

    bool target_reached(double threshold = 1.0) const
    {
        return std::abs(remaining_distance()) < threshold;
    }

  private:
    double position_;
    double velocity_;
    double target_position_;
};

int main(void)
{
    std::cout << "===========================================\n";
    std::cout << "   Tracker Chain Test\n";
    std::cout << "===========================================\n\n";

    // Create ControllersIO
    ControllersIO io;

    // ========================================================================
    // CONTROLLER 1: ProfileTrackerController
    // ========================================================================
    ProfileTrackerControllerIOKeys profile_keys = {.pose_error = "linear_pose_error",
                                                   .current_speed = "current_linear_speed",
                                                   .recompute_profile = "recompute_profile",
                                                   .tracker_velocity = "linear_tracker_velocity",
                                                   .tracking_error = "linear_tracking_error",
                                                   .profile_complete = "linear_profile_complete",
                                                   .target_speed = ""};

    ProfileTrackerControllerParameters profile_params(10.0, // max_speed (mm/period)
                                                      1.0,  // acceleration (mm/period²)
                                                      1.0,  // deceleration (mm/period²)
                                                      true  // must_stop_at_end
    );

    ProfileTrackerController profile_controller(profile_keys, profile_params);

    // ========================================================================
    // CONTROLLER 2: SimplePID (for tracking error correction)
    // ========================================================================
    // We use SimplePID for this test to avoid protobuf dependencies
    SimplePID pid(0.1, 0.0, 0.0); // P controller

    // ========================================================================
    // CONTROLLER 3: TrackerCombinerController
    // ========================================================================
    TrackerCombinerControllerIOKeys combiner_keys = {.tracker_velocity = "linear_tracker_velocity",
                                                     .feedback_correction =
                                                         "linear_feedback_correction",
                                                     .speed_order = "linear_speed_order"};

    TrackerCombinerControllerParameters combiner_params;

    TrackerCombinerController combiner_controller(combiner_keys, combiner_params);

    // ========================================================================
    // Robot Simulator
    // ========================================================================
    RobotSimulator robot;
    robot.set_target(1000.0); // Move 1000mm

    // Initialize IO with target
    io.set("linear_pose_error", robot.remaining_distance());
    io.set("current_linear_speed", robot.velocity());
    io.set("recompute_profile", true);

    std::cout << "Target: 1000 mm\n";
    std::cout << "Max speed: 10 mm/period\n";
    std::cout << "Acceleration: 1 mm/period²\n";
    std::cout << "Deceleration: 1 mm/period²\n";
    std::cout << "PID: Kp=0.1, Ki=0.0, Kd=0.0\n\n";

    std::cout << "Period | Position | Velocity | Remaining | FF Vel | Tracking Err | FB Corr | "
                 "Speed Order\n";
    std::cout
        << "-------|----------|----------|-----------|--------|--------------|---------|-------"
           "-----\n";

    // ========================================================================
    // Simulation Loop
    // ========================================================================
    for (int period = 0; period < 150 && !robot.target_reached(); period++) {
        // Update IO with current robot state
        io.set("linear_pose_error", robot.remaining_distance());
        io.set("current_linear_speed", robot.velocity());

        // STEP 1: ProfileTrackerController
        profile_controller.execute(io);

        // STEP 2: PID on tracking error (manual, since we use SimplePID)
        double tracking_error = 0.0;
        if (auto opt = io.get_as<double>("linear_tracking_error")) {
            tracking_error = *opt;
        }
        double feedback_correction = pid.compute(tracking_error);
        io.set("linear_feedback_correction", feedback_correction);

        // STEP 3: TrackerCombinerController
        combiner_controller.execute(io);

        // Read final velocity command
        double velocity_command = 0.0;
        if (auto opt = io.get_as<double>("linear_speed_order")) {
            velocity_command = *opt;
        }

        // Read intermediate values for display
        double ff_velocity = 0.0;
        if (auto opt = io.get_as<double>("linear_tracker_velocity")) {
            ff_velocity = *opt;
        }

        // Update robot simulator
        robot.update(velocity_command);

        // Print every 10 periods
        if (period % 10 == 0) {
            printf("%6d | %8.2f | %8.2f | %9.2f | %6.2f | %12.2f | %7.2f | %11.2f\n", period,
                   robot.position(), robot.velocity(), robot.remaining_distance(), ff_velocity,
                   tracking_error, feedback_correction, velocity_command);
        }

        // Clear recompute_profile flag after first iteration
        if (period == 0) {
            io.set("recompute_profile", false);
        }
    }

    // Final state
    printf("\n✓ Target reached!\n");
    printf("Final position: %.2f mm\n", robot.position());
    printf("Final velocity: %.2f mm/period\n", robot.velocity());
    printf("Final error: %.2f mm\n", robot.remaining_distance());

    std::cout << "\n===========================================\n";
    std::cout << "   Test completed!\n";
    std::cout << "===========================================\n";

    return 0;
}
