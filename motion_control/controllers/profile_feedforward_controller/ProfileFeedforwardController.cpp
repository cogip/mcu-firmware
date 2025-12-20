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

// ETL includes
#include "etl/absolute.h"

// Project includes
#include "log.h"
#include "profile_feedforward_controller/ProfileFeedforwardController.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

ProfileFeedforwardController::ProfileFeedforwardController(
    const ProfileFeedforwardControllerIOKeys& keys,
    const ProfileFeedforwardControllerParameters& parameters, etl::string_view name)
    : Controller<ProfileFeedforwardControllerIOKeys, ProfileFeedforwardControllerParameters>(
          keys, parameters, name),
      profile_(), period_(0)
{
}

void ProfileFeedforwardController::execute(ControllersIO& io)
{
    DEBUG("Execute ProfileFeedforwardController [%s]\n", keys_.pose_error.data());

    // Check if profile invalidation requested
    if (!keys_.invalidate_profile.empty()) {
        bool invalidate_profile = false;
        if (auto opt = io.get_as<bool>(keys_.invalidate_profile)) {
            invalidate_profile = *opt;
            DEBUG("[%s] Read invalidate_profile=%d from key '%s'\n", keys_.pose_error.data(),
                  invalidate_profile, keys_.invalidate_profile.data());
        } else {
            DEBUG("[%s] Could not read invalidate_profile from key '%s'\n", keys_.pose_error.data(),
                  keys_.invalidate_profile.data());
        }
        if (invalidate_profile) {
            DEBUG("[%s] Profile invalidated\n", keys_.pose_error.data());
            profile_.reset();
        }
    } else {
        DEBUG("[%s] No invalidate_profile key configured\n", keys_.pose_error.data());
    }

    // Check if profile recomputation requested (from PoseStraightFilter state transitions)
    bool recompute_profile = false;
    if (auto opt = io.get_as<bool>(keys_.recompute_profile)) {
        recompute_profile = *opt;
    }

    // If profile is not initialized, output pose_error as tracking_error
    // This allows position control without feedforward when no profile is running
    // Skip this if recompute is requested (recompute takes priority)
    if (!recompute_profile && !profile_.is_initialized()) {
        // Read pose error for tracking
        float pose_error = 0.0f;
        if (auto opt = io.get_as<float>(keys_.pose_error)) {
            pose_error = *opt;
        }
        // No feedforward velocity when profile is invalidated
        io.set(keys_.feedforward_velocity, 0.0f);
        // Use pose_error as tracking_error for position control
        io.set(keys_.tracking_error, pose_error);
        DEBUG("[%s] Profile not ready: tracking_error=%.2f for position control\n",
              keys_.pose_error.data(), pose_error);
        return;
    }

    // Read pose error (calculated by PoseStraightFilter)
    float pose_error = 0.0f;
    if (auto opt = io.get_as<float>(keys_.pose_error)) {
        pose_error = *opt;
    } else {
        LOG_ERROR("ProfileFeedforwardController: pose_error not available\n");
        io.set(keys_.feedforward_velocity, 0.0f);
        io.set(keys_.tracking_error, 0.0f);
        return;
    }

    // Generate new profile if requested (triggered by PoseStraightFilter state transitions)
    if (recompute_profile) {
        DEBUG("ProfileFeedforward[%s]: RECOMPUTE requested, pose_error=%.2f\n",
              keys_.pose_error.data(), static_cast<double>(pose_error));
        // Use pose error as target distance (signed - TrapezoidalProfile handles direction)
        float target_distance = pose_error;

        // Read current speed (use absolute value, clamped to max_speed)
        float current_speed = 0.0f;
        if (auto opt = io.get_as<float>(keys_.current_speed)) {
            current_speed = etl::absolute(*opt);
            // Clamp to max_speed to prevent corrupted odometry from creating invalid profiles
            if (current_speed > parameters_.max_speed()) {
                LOG_WARNING(
                    "ProfileFeedforwardController: current_speed %.2f clamped to max %.2f\n",
                    current_speed, parameters_.max_speed());
                current_speed = parameters_.max_speed();
            }
        }

        // Generate optimal profile with signed distance (TrapezoidalProfile handles direction)
        uint32_t total_periods = profile_.generate_optimal_profile(
            current_speed, target_distance, parameters_.acceleration(), parameters_.deceleration(),
            parameters_.max_speed(), parameters_.must_stop_at_end());

        if (total_periods == 0) {
            LOG_WARNING("ProfileFeedforwardController: No profile generated (distance=%.2f)\n",
                        target_distance);
            io.set(keys_.feedforward_velocity, 0.0f);
            io.set(keys_.tracking_error, 0.0f);
            io.set(keys_.recompute_profile, false); // Clear flag even on failure
            profile_.reset();
            return;
        }

        DEBUG("[%s] Generated profile: %" PRIu32 " periods, distance=%.2f\n",
              keys_.pose_error.data(), total_periods, target_distance);

        // Reset period counter
        period_ = 0;

        // Clear recompute flag to prevent regenerating the profile every cycle
        io.set(keys_.recompute_profile, false);
    }

    // Check if profile is complete
    bool profile_complete = (period_ >= profile_.total_periods());

    // Output profile_complete flag if key is configured
    if (!keys_.profile_complete.empty()) {
        io.set(keys_.profile_complete, profile_complete);
    }

    // When profile is complete, continue with PID-only control using pose_error
    // Don't invalidate - let the state machine handle the transition
    // The feedforward velocity is 0 (profile done), but tracking_error = pose_error
    // allows the PID to finish bringing the robot to target
    // NOTE: Use pose_error directly WITHOUT direction_sign_ because pose_error
    // from PoseStraightFilter is already properly signed (negative for backward motion)
    if (profile_complete) {
        io.set(keys_.feedforward_velocity, 0.0f);
        io.set(keys_.tracking_error, pose_error);
        DEBUG("[%s] Profile complete: PID-only control with tracking_error=%.2f\n",
              keys_.pose_error.data(), pose_error);
        return;
    }

    // Compute feedforward velocity from profile (signed - negative for backward movement)
    float feedforward_velocity = profile_.compute_theoretical_velocity(period_);

    // Compute theoretical remaining distance from profile (signed)
    float theoretical_remaining = profile_.compute_theoretical_remaining_distance(period_);

    // Compute tracking error: actual - theoretical (both signed, same direction)
    // If actual > theoretical: we are behind schedule (positive error for forward, negative for
    // backward) If actual < theoretical: we are ahead of schedule
    float tracking_error = pose_error - theoretical_remaining;

    DEBUG("[%s] Period %" PRIu32 ": feedforward_velocity=%.2f, pose_error=%.2f, "
          "theoretical_remaining=%.2f, tracking_error=%.2f\n",
          keys_.pose_error.data(), period_, feedforward_velocity, pose_error, theoretical_remaining,
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
