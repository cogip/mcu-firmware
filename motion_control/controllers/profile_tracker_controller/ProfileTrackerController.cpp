// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    profile_tracker_controller
/// @{
/// @file
/// @brief      Profile Tracker controller implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// System includes
#include <cstdio>
#include <inttypes.h>

// ETL includes
#include "etl/algorithm.h"

// Project includes
#include "log.h"
#include "profile_tracker_controller/ProfileTrackerController.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

ProfileTrackerController::ProfileTrackerController(
    const ProfileTrackerControllerIOKeys& keys,
    const ProfileTrackerControllerParameters& parameters, etl::string_view name)
    : Controller<ProfileTrackerControllerIOKeys, ProfileTrackerControllerParameters>(
          keys, parameters, name),
      profile_(), period_(0)
{
}

bool ProfileTrackerController::generate_profile(float target_distance, float initial_speed,
                                                float max_speed, bool must_stop)
{
    uint32_t total_periods = profile_.generate_optimal_profile(
        initial_speed, target_distance, parameters_.acceleration(), parameters_.deceleration(),
        max_speed, must_stop);

    if (total_periods == 0) {
        LOG_WARNING("ProfileTrackerController: No profile generated (distance=%.2f)\n",
                    target_distance);
        profile_.reset();
        return false;
    }

    period_ = 0;
    return true;
}

void ProfileTrackerController::execute_speed_mode(ControllersIO& io)
{
    DEBUG("Execute ProfileTrackerController [speed_mode]\n");

    // Check if profile recomputation requested
    bool recompute_profile = false;
    if (auto opt = io.get_as<bool>(keys_.recompute_profile)) {
        recompute_profile = *opt;
    }

    if (recompute_profile) {
        // Read signed target speed
        float target_speed_signed = 0.0f;
        if (auto opt = io.get_as<float>(keys_.target_speed)) {
            target_speed_signed = *opt;
        }

        // Read duration in periods
        uint32_t duration = 0;
        if (auto opt = io.get_as<int>(keys_.duration_periods)) {
            duration = static_cast<uint32_t>(*opt);
        }

        if (duration == 0 || target_speed_signed == 0.0f) {
            DEBUG("[speed_mode] No profile: duration=%" PRIu32 " speed=%.2f\n", duration,
                  target_speed_signed);
            io.set(keys_.tracker_velocity, 0.0f);
            profile_.reset();
            return;
        }

        float max_speed = etl::min(parameters_.max_speed(), etl::absolute(target_speed_signed));
        float direction = (target_speed_signed >= 0.0f) ? 1.0f : -1.0f;

        // Compute distance from speed + duration (accounts for accel/decel ramps)
        float target_distance = direction * TrapezoidalProfile::compute_distance_for_duration(
                                                max_speed, parameters_.acceleration(),
                                                parameters_.deceleration(), duration, true);

        DEBUG("[speed_mode] RECOMPUTE: speed=%.2f duration=%" PRIu32 " distance=%.2f\n", max_speed,
              duration, target_distance);

        if (!generate_profile(target_distance, 0.0f, max_speed, true)) {
            io.set(keys_.tracker_velocity, 0.0f);
            return;
        }

        DEBUG("[speed_mode] Generated profile: %" PRIu32 " periods\n", profile_.total_periods());
    }

    if (!profile_.is_initialized()) {
        io.set(keys_.tracker_velocity, 0.0f);
        return;
    }

    // Profile complete
    if (period_ >= profile_.total_periods()) {
        io.set(keys_.tracker_velocity, 0.0f);
        DEBUG("[speed_mode] Profile complete\n");
        return;
    }

    // Compute velocity from trapezoidal profile
    float velocity = profile_.compute_theoretical_velocity(period_);

    DEBUG("[speed_mode] Period %" PRIu32 ": velocity=%.2f\n", period_, velocity);

    io.set(keys_.tracker_velocity, velocity);
    period_ += parameters_.period_increment();
}

void ProfileTrackerController::execute(ControllersIO& io)
{
    if (parameters_.speed_mode()) {
        execute_speed_mode(io);
        return;
    }

    DEBUG("Execute ProfileTrackerController [%s]\n", keys_.pose_error.data());

    // Check if profile recomputation requested (from PoseStraightFilter state transitions)
    bool recompute_profile = false;
    if (auto opt = io.get_as<bool>(keys_.recompute_profile)) {
        recompute_profile = *opt;
    }

    // If profile is not initialized, output pose_error as tracking_error
    // This allows position control without tracker when no profile is running
    // Skip this if recompute is requested (recompute takes priority)
    if (!recompute_profile && !profile_.is_initialized()) {
        // Read pose error for tracking
        float pose_error = 0.0f;
        if (auto opt = io.get_as<float>(keys_.pose_error)) {
            pose_error = *opt;
        }
        // No tracker velocity when profile is invalidated
        io.set(keys_.tracker_velocity, 0.0f);
        // Use pose_error as tracking_error for position control
        io.set(keys_.tracking_error, pose_error);
        // Profile not running, so not complete
        if (!keys_.profile_complete.empty()) {
            io.set(keys_.profile_complete, false);
        }
        DEBUG("[%s] Profile not ready: tracking_error=%.2f for position control\n",
              keys_.pose_error.data(), pose_error);
        return;
    }

    // Read pose error (calculated by PoseStraightFilter)
    float pose_error = 0.0f;
    if (auto opt = io.get_as<float>(keys_.pose_error)) {
        pose_error = *opt;
    } else {
        LOG_ERROR("ProfileTrackerController: pose_error not available\n");
        io.set(keys_.tracker_velocity, 0.0f);
        io.set(keys_.tracking_error, 0.0f);
        if (!keys_.profile_complete.empty()) {
            io.set(keys_.profile_complete, false);
        }
        return;
    }

    // Read current speed (keep sign for proper profile generation)
    float current_speed = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_speed)) {
        current_speed = *opt;
    }

    // Get effective max speed: use target_speed from IO if available, otherwise use parameter
    float max_speed = parameters_.max_speed();
    if (!keys_.target_speed.empty()) {
        if (auto opt = io.get_as<float>(keys_.target_speed)) {
            // Use the minimum of parameter max and target speed from path
            max_speed = etl::min(max_speed, *opt);
            DEBUG("[%s] Using target_speed from IO: %.2f\n", keys_.pose_error.data(), max_speed);
        }
    }

    // Generate new profile if requested (triggered by PoseStraightFilter state transitions)
    if (recompute_profile) {
        DEBUG("[ProfileFF %s] RECOMPUTE pose_error=%.2f current_speed=%.2f\n",
              keys_.pose_error.data(), static_cast<double>(pose_error),
              static_cast<double>(current_speed));

        if (!generate_profile(pose_error, current_speed, max_speed,
                              parameters_.must_stop_at_end())) {
            io.set(keys_.tracker_velocity, 0.0f);
            io.set(keys_.tracking_error, 0.0f);
            if (!keys_.profile_complete.empty()) {
                io.set(keys_.profile_complete, false);
            }
            return;
        }

        DEBUG("[%s] Generated profile: %" PRIu32 " periods, distance=%.2f\n",
              keys_.pose_error.data(), profile_.total_periods(), pose_error);
    }

    // Check if pose_error sign changed vs profile target (e.g., motion_dir changed between cycles)
    // If signs are opposite, regenerate profile for the new direction
    float profile_target = profile_.target_distance();
    if ((pose_error > 0.0f && profile_target < 0.0f) ||
        (pose_error < 0.0f && profile_target > 0.0f)) {
        DEBUG("[%s] Sign mismatch: pose_error=%.2f vs profile_target=%.2f, regenerating profile\n",
              keys_.pose_error.data(), pose_error, profile_target);

        if (!generate_profile(pose_error, current_speed, max_speed,
                              parameters_.must_stop_at_end())) {
            // Can't generate profile - fall back to PID-only
            io.set(keys_.tracker_velocity, 0.0f);
            io.set(keys_.tracking_error, pose_error);
            if (!keys_.profile_complete.empty()) {
                io.set(keys_.profile_complete, false);
            }
            return;
        }

        DEBUG("[%s] Regenerated profile: %" PRIu32 " periods, distance=%.2f\n",
              keys_.pose_error.data(), profile_.total_periods(), pose_error);
    }

    // Check if profile is complete
    bool profile_complete = (period_ >= profile_.total_periods());

    // Output profile_complete flag if key is configured
    if (!keys_.profile_complete.empty()) {
        io.set(keys_.profile_complete, profile_complete);
    }

    // When profile is complete, continue with PID-only control using pose_error
    // Don't invalidate - let the state machine handle the transition
    // The tracker velocity is 0 (profile done), but tracking_error = pose_error
    // allows the PID to finish bringing the robot to target
    // NOTE: Use pose_error directly WITHOUT direction_sign_ because pose_error
    // from PoseStraightFilter is already properly signed (negative for backward motion)
    if (profile_complete) {
        io.set(keys_.tracker_velocity, 0.0f);
        io.set(keys_.tracking_error, pose_error);
        DEBUG("[%s] Profile complete: PID-only control with tracking_error=%.2f\n",
              keys_.pose_error.data(), pose_error);
        return;
    }

    // Compute tracker velocity from profile (signed - negative for backward movement)
    float tracker_velocity = profile_.compute_theoretical_velocity(period_);

    // Compute theoretical remaining distance from profile (signed)
    float theoretical_remaining = profile_.compute_theoretical_remaining_distance(period_);

    // Compute tracking error: actual - theoretical (both signed, same direction)
    // If actual > theoretical: we are behind schedule (positive error for forward, negative for
    // backward) If actual < theoretical: we are ahead of schedule
    float tracking_error = pose_error - theoretical_remaining;

    DEBUG("[%s] Period %" PRIu32 ": tracker_velocity=%.2f, pose_error=%.2f, "
          "theoretical_remaining=%.2f, tracking_error=%.2f\n",
          keys_.pose_error.data(), period_, tracker_velocity, pose_error, theoretical_remaining,
          tracking_error);

    // Write outputs
    io.set(keys_.tracker_velocity, tracker_velocity);
    io.set(keys_.tracking_error, tracking_error);

    // Increment period counter (by period_increment for throttled controllers)
    period_ += parameters_.period_increment();
}

} // namespace motion_control

} // namespace cogip

/// @}
