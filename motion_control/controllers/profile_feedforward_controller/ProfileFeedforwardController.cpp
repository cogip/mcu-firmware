// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    profile_feedforward_controller
/// @{
/// @file
/// @brief      Profile Feedforward controller implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cstdio>

// Project includes
#include "log.h"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

ProfileFeedforwardController::ProfileFeedforwardController(
    const ProfileFeedforwardControllerIOKeys& keys,
    const ProfileFeedforwardControllerParameters& parameters)
    : Controller<ProfileFeedforwardControllerIOKeys, ProfileFeedforwardControllerParameters>(keys, parameters),
      profile_(),
      period_(0),
      profile_ready_(false)
{
}

void ProfileFeedforwardController::execute(ControllersIO& io)
{
    DEBUG("Execute ProfileFeedforwardController");

    // Check if new target requested
    bool new_target = false;
    if (auto opt = io.get_as<bool>(keys_.new_target)) {
        new_target = *opt;
    }

    // Generate new profile if requested
    if (new_target) {
        // Read target distance (distance remaining)
        double target_distance = 0.0;
        if (auto opt = io.get_as<double>(keys_.pose_error)) {
            target_distance = *opt;
        }
        else {
            LOG_ERROR("ProfileFeedforwardController: pose_error not available\n");
            io.set(keys_.feedforward_velocity, 0.0);
            io.set(keys_.tracking_error, 0.0);
            profile_ready_ = false;
            return;
        }

        // Read current speed
        double current_speed = 0.0;
        if (auto opt = io.get_as<double>(keys_.current_speed)) {
            current_speed = *opt;
        }

        // Generate optimal profile
        uint32_t total_periods = profile_.generate_optimal_profile(
            current_speed,
            target_distance,
            parameters_.acceleration(),
            parameters_.deceleration(),
            parameters_.max_speed(),
            parameters_.must_stop_at_end());

        if (total_periods == 0) {
            LOG_WARNING("ProfileFeedforwardController: No profile generated (distance=%.2f)\n", target_distance);
            io.set(keys_.feedforward_velocity, 0.0);
            io.set(keys_.tracking_error, 0.0);
            profile_ready_ = false;
            return;
        }

        DEBUG("Generated profile: %" PRIu32 " periods, target_distance=%.2f\n", total_periods,
              target_distance);

        // Reset period counter
        period_ = 0;
        profile_ready_ = true;

        // Clear new_target flag
        io.set(keys_.new_target, false);
    }

    // If no profile ready, output zero
    if (!profile_ready_) {
        io.set(keys_.feedforward_velocity, 0.0);
        io.set(keys_.tracking_error, 0.0);
        return;
    }

    // Compute feedforward velocity from profile
    double feedforward_velocity = profile_.compute_theoretical_velocity(period_);

    // Compute theoretical remaining distance from profile
    double theoretical_remaining = profile_.compute_theoretical_remaining_distance(period_);

    // Read actual remaining distance (updated by odometry)
    double actual_remaining = 0.0;
    if (auto opt = io.get_as<double>(keys_.pose_error)) {
        actual_remaining = *opt;
    }
    else {
        LOG_WARNING("ProfileFeedforwardController: pose_error not available during execution\n");
    }

    // Compute tracking error: actual - theoretical
    // If actual > theoretical: we are behind schedule (positive error)
    // If actual < theoretical: we are ahead of schedule (negative error)
    double tracking_error = actual_remaining - theoretical_remaining;

    DEBUG("Period %" PRIu32 ": ff_vel=%.2f, actual_rem=%.2f, theo_rem=%.2f, track_err=%.2f\n",
          period_,
          feedforward_velocity,
          actual_remaining,
          theoretical_remaining,
          tracking_error);

    // Write outputs
    io.set(keys_.feedforward_velocity, feedforward_velocity);
    io.set(keys_.tracking_error, tracking_error);

    // Increment period counter
    period_++;
}

} // namespace motion_control

} // namespace cogip

/// @}
