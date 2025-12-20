// System includes
#include "etl/absolute.h"
#include <cmath>

// Set log level for this file (only show state transitions - WARNING level)

// Project includes
#include "cogip_defs/Polar.hpp"
#include "cogip_defs/Pose.hpp"
#include "log.h"
#include "path/MotionDirection.hpp"
#include "pose_straight_filter/PoseStraightFilter.hpp"
#include "trigonometry.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

void PoseStraightFilter::execute(ControllersIO& io)
{
    DEBUG("Execute PoseStraightFilter\n");

    // Read current pose coordinates and orientation
    float current_pose_x = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_x)) {
        current_pose_x = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.current_pose_x.data(), current_pose_x);
    }
    float current_pose_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_y)) {
        current_pose_y = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.current_pose_y.data(), current_pose_y);
    }
    float current_pose_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_O)) {
        current_pose_O = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.current_pose_O.data(), current_pose_O);
    }
    cogip_defs::Pose current_pose(current_pose_x, current_pose_y, current_pose_O);

    // Read target pose coordinates and orientation
    float target_pose_x = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose_x)) {
        target_pose_x = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.target_pose_x.data(), target_pose_x);
    }
    float target_pose_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose_y)) {
        target_pose_y = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.target_pose_y.data(), target_pose_y);
    }
    float target_pose_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose_O)) {
        target_pose_O = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.target_pose_O.data(), target_pose_O);
    }
    cogip_defs::Pose target_pose(target_pose_x, target_pose_y, target_pose_O);

    // Read current linear and angular speeds
    float curr_lin = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_linear_speed)) {
        curr_lin = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.current_linear_speed.data(), curr_lin);
    }
    float curr_ang = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_angular_speed)) {
        curr_ang = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.current_angular_speed.data(), curr_ang);
    }
    const cogip_defs::Polar current_speed(curr_lin, curr_ang);

    // Read target linear and angular speeds
    float target_pose_lin = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_linear_speed)) {
        target_pose_lin = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.target_linear_speed.data(), target_pose_lin);
    }
    float target_pose_ang = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_angular_speed)) {
        target_pose_ang = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f\n",
                    keys_.target_angular_speed.data(), target_pose_ang);
    }
    cogip_defs::Polar target_speed(target_pose_lin, target_pose_ang);

    // Read motion direction mode
    cogip::path::motion_direction motion_dir = cogip::path::motion_direction::bidirectional;
    if (auto opt = io.get_as<int>(keys_.motion_direction)) {
        motion_dir = static_cast<cogip::path::motion_direction>(*opt);
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value bidirectional\n",
                    keys_.motion_direction.data());
    }

    // Persist direction choice when switching to MOVE_TO_POSITION state
    static bool force_can_reverse = false;
    static cogip_defs::Pose prev_target(INT32_MAX, INT32_MAX, INT32_MAX);
    // Reference orientation saved when entering MOVE_TO_POSITION for heading maintenance
    static float reference_orientation = 0.0f;
    // Initial direction sign: +1 for forward, -1 for backward
    // Saved when entering MOVE_TO_POSITION to maintain consistent direction
    static float initial_direction_sign = 1.0f;

    // Recompute profile signals - set to true only on specific state transitions
    bool linear_recompute_profile = false;
    bool linear_invalidate_profile = false;
    bool angular_recompute_profile = false;
    bool angular_invalidate_profile = false;

    // Static variables for ROTATE_TO_FINAL_ANGLE oscillation detection
    // Declared here so they can be reset on target change
    static float prev_angular_error = 0.0f;
    static bool first_entry = true;

    if ((target_pose_x != prev_target.x()) || (target_pose_y != prev_target.y()) ||
        (target_pose_O != prev_target.O())) {
        force_can_reverse = false;
        prev_target = target_pose;
        // Reset state machine to initial state on new target
        current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION;
        // Reset ROTATE_TO_FINAL_ANGLE oscillation detection variables
        prev_angular_error = 0.0f;
        first_entry = true;
        // New external target -> recompute angular profile for initial rotation
        angular_recompute_profile = true;
        // Invalidate linear profile - will be recomputed when entering MOVE_TO_POSITION
        linear_invalidate_profile = true;
        DEBUG("New target detected, reset state to ROTATE_TO_DIRECTION\n");
    }

    // Apply motion direction constraints
    bool can_reverse = false;
    bool must_reverse = false;

    switch (motion_dir) {
    case cogip::path::motion_direction::bidirectional:
        // Allow optimal choice: reverse if angle > 90°
        can_reverse = true;
        break;
    case cogip::path::motion_direction::forward_only:
        // Never reverse
        break;
    case cogip::path::motion_direction::backward_only:
        // Always reverse
        must_reverse = true;
        break;
    default:
        LOG_ERROR("ERROR: motion direction invalid - should never happen\n");
    }

    // Force can_reverse to true once motion has started (only for bidirectional mode)
    // For backward_only mode, we must keep must_reverse active
    if (force_can_reverse && motion_dir == cogip::path::motion_direction::bidirectional) {
        can_reverse = true;
    }

    // Compute pose error as polar difference
    cogip_defs::Polar pos_err = target_pose - current_pose;

    // Store raw angle before any reverse operation for overshoot detection
    // For backward motion: |raw_ang| ≈ 180° means robot is behind target (correct)
    //                      |raw_ang| < 135° means robot has crossed/overshot target
    const float raw_angular_error = pos_err.angle();

    DEBUG("pos_err BEFORE reverse: dist=%.2f ang=%.2f\n", static_cast<double>(pos_err.distance()),
          static_cast<double>(pos_err.angle()));
    DEBUG("motion_dir=%d can_reverse=%d must_reverse=%d\n", static_cast<int>(motion_dir),
          can_reverse, must_reverse);

    // Apply direction based on motion mode:
    // - For backward_only (must_reverse): always reverse to get negative distance (backward motion)
    //   The angle after reverse will be the optimal heading (away from target)
    // - For bidirectional (can_reverse): reverse only if |angle| > 90° (shorter rotation path)
    if (must_reverse) {
        // Always reverse for backward_only mode - ensures negative distance
        DEBUG("Reverse error as must_reverse (backward_only mode)\n");
        pos_err.reverse();
    } else if (can_reverse && etl::absolute(pos_err.angle()) > 90.0f) {
        // Target is behind and we can choose -> reverse to go backward (shorter rotation)
        DEBUG("Reverse error as can reverse (angle > 90)\n");
        pos_err.reverse();
    }

    DEBUG("pos_err AFTER reverse: dist=%.2f ang=%.2f\n", static_cast<double>(pos_err.distance()),
          static_cast<double>(pos_err.angle()));

    bool no_angular_limit_flag = false;
    bool no_linear_limit_flag = false;

    const float linear_threshold = parameters_.linear_threshold();
    const float angular_threshold = parameters_.angular_threshold();
    const float angular_intermediate_threshold = parameters_.angular_intermediate_threshold();

    const float absolute_linear_pose_error = etl::absolute(pos_err.distance());
    const float absolute_angular_pose_error = etl::absolute(pos_err.angle());

    // State transitions
    if ((current_state_ == PoseStraightFilterState::ROTATE_TO_DIRECTION) &&
        (absolute_linear_pose_error <= linear_threshold)) {
        current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
    }

    switch (current_state_) {
    case PoseStraightFilterState::ROTATE_TO_DIRECTION:
        DEBUG("ROTATE_TO_DIRECTION\n");
        // Keep linear profile invalidated during initial rotation
        linear_invalidate_profile = true;
        // Force linear target speed to 0 during initial rotation
        target_speed.set_distance(0.0f);
        if (absolute_angular_pose_error > angular_intermediate_threshold) {
            // Still rotating to face target direction
        } else {
            // Save the initial direction sign for use during MOVE_TO_POSITION
            // This determines if we started going forward (+1) or backward (-1)
            initial_direction_sign = (pos_err.distance() >= 0) ? 1.0f : -1.0f;
            // Linear speed comes from feedforward profile, not from here
            target_speed.set_distance(0.0f);
            force_can_reverse = true;
            current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
            // Save the TARGET direction (current orientation + remaining angular error) as
            // reference This is the ideal heading the robot should maintain during linear motion
            reference_orientation = limit_angle_deg(current_pose_O + pos_err.angle());
            // Initialize overshoot detection: target is in front if going forward, behind if
            // backward
            target_is_in_front_ = (initial_direction_sign > 0);
            // Transition to MOVE_TO_POSITION -> recompute linear profile and invalidate angular
            // profile
            linear_recompute_profile = true;
            angular_invalidate_profile = true;
            DEBUG(
                "Transition to MOVE_TO_POSITION, reference_orientation=%.2f (cur=%.2f + err=%.2f), "
                "initial_direction_sign=%.0f, requesting linear profile recompute\n",
                static_cast<double>(reference_orientation), static_cast<double>(current_pose_O),
                static_cast<double>(pos_err.angle()), static_cast<double>(initial_direction_sign));
        }
        break;

    case PoseStraightFilterState::MOVE_TO_POSITION: {
        // During MOVE_TO_POSITION, the robot is BIDIRECTIONAL regardless of initial motion_dir.
        // This allows natural correction if the robot overshoots the target.
        //
        // The linear error sign convention must match how the profile was generated:
        // - If we started FORWARD (initial_direction_sign = +1):
        //   - Target in front (|raw_ang| < 90°) → positive error (keep going forward)
        //   - Target behind (|raw_ang| > 90°) → negative error (need to go backward)
        // - If we started BACKWARD (initial_direction_sign = -1):
        //   - Target behind (|raw_ang| > 90°) → negative error (keep going backward)
        //   - Target in front (|raw_ang| < 90°) → positive error (need to go forward)

        // Get raw distance (always positive)
        cogip_defs::Polar raw_pos_err = target_pose - current_pose;
        float raw_distance = raw_pos_err.distance();

        // Determine if target is in front or behind based on raw angle
        // Target in front: |raw_angle| < 90°
        // Target behind: |raw_angle| > 90°
        bool target_is_in_front = (etl::absolute(raw_angular_error) < 90.0f);

        // Detect overshoot: if target_is_in_front changes during MOVE_TO_POSITION,
        // the robot crossed the target. target_is_in_front_ is initialized
        // when entering MOVE_TO_POSITION state (in ROTATE_TO_DIRECTION transition).
        bool overshoot_detected = (target_is_in_front != target_is_in_front_);
        target_is_in_front_ = target_is_in_front;

        // Compute linear error based on target direction:
        // - Target in front (|raw_ang| < 90°): positive error → robot should go forward
        // - Target behind (|raw_ang| > 90°): negative error → robot should go backward
        //
        // This sign convention works for both forward and backward initial motion:
        // - Forward start: target in front → positive → keep going forward ✓
        // - Backward start: target behind → negative → keep going backward ✓
        // - Overshoot: sign flips naturally, allowing correction in opposite direction
        //
        // The signed error is used by both ProfileFeedforwardController (for profile
        // generation and tracking) and PosePIDController (for feedback correction).
        float linear_error = target_is_in_front ? raw_distance : -raw_distance;

        // Update pos_err with the bidirectional linear error
        pos_err.set_distance(linear_error);

        // Maintain heading by tracking the reference orientation
        float heading_error = limit_angle_deg(reference_orientation - current_pose_O);
        pos_err.set_angle(heading_error);

        // Keep angular profile invalidated during entire MOVE_TO_POSITION state
        angular_invalidate_profile = true;

        DEBUG("MOVE_TO_POSITION: lin_err=%.2f heading_err=%.2f (ref=%.2f cur=%.2f) raw_ang=%.2f "
              "init_dir=%.0f\n",
              static_cast<double>(linear_error), static_cast<double>(heading_error),
              static_cast<double>(reference_orientation), static_cast<double>(current_pose_O),
              static_cast<double>(raw_angular_error), static_cast<double>(initial_direction_sign));

        // Transition when close enough to target OR overshoot detected
        if (raw_distance <= linear_threshold || overshoot_detected) {
            current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
            // Transition to ROTATE_TO_FINAL_ANGLE -> recompute angular profile and invalidate
            // linear profile
            angular_recompute_profile = true;
            linear_invalidate_profile = true;
            // IMPORTANT: Clear angular_invalidate_profile so recompute takes effect
            angular_invalidate_profile = false;
            // Update angular error to final angle error (not heading error)
            if (!parameters_.bypass_final_orientation()) {
                pos_err.set_angle(limit_angle_deg(target_pose_O - current_pose_O));
            } else {
                pos_err.set_angle(0.0f);
            }
            DEBUG("Transition to ROTATE_TO_FINAL_ANGLE, requesting angular profile recompute\n");
        }
    } break;

    case PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE:
        DEBUG("ROTATE_TO_FINAL_ANGLE\n");
        {
            // Keep linear profile invalidated during final rotation
            linear_invalidate_profile = true;
            // Force linear target speed to 0 during final rotation
            target_speed.set_distance(0.0f);

            if (!parameters_.bypass_final_orientation()) {
                pos_err.set_angle(limit_angle_deg(target_pose_O - current_pose_O));
            } else {
                pos_err.set_angle(0.0f);
            }

            float current_angular_error = pos_err.angle();

            // Detect completion conditions:
            // 1. Sign change: angular error crosses zero (prev and current have opposite signs)
            //    This indicates oscillation around the target angle - we're close enough
            // 2. Within threshold: angular error is small enough to consider target reached
            // Note: first_entry check prevents false positive on first iteration
            bool sign_changed = !first_entry && (prev_angular_error * current_angular_error < 0.0f);
            bool within_threshold = etl::absolute(current_angular_error) <= angular_threshold;

            if (within_threshold || sign_changed) {
                // Target angle reached - move to FINISHED state
                current_state_ = PoseStraightFilterState::FINISHED;
                first_entry = true;
            } else {
                // Continue rotating - save current error for next iteration's oscillation detection
                prev_angular_error = current_angular_error;
                first_entry = false;
            }
        }
        break;

    case PoseStraightFilterState::FINISHED:
        DEBUG("FINISHED\n");
        // Force target speeds to 0 to prevent any motion
        target_speed.set_distance(0.0f);
        target_speed.set_angle(0.0f);
        // Keep profiles invalidated in FINISHED state
        linear_invalidate_profile = true;
        angular_invalidate_profile = true;
        break;
    }

    // Note: Deceleration is handled by the feedforward profile controllers
    // which generate trapezoidal velocity profiles. No additional deceleration
    // logic is needed here.

    // Write linear pose error
    io.set(keys_.linear_pose_error, pos_err.distance());

    // Write linear target speed as absolute
    io.set(keys_.linear_target_speed, etl::absolute(target_speed.distance()));

    // Write linear speed filter flag
    io.set(keys_.linear_speed_filter_flag, no_linear_limit_flag);

    // Write angular pose error
    io.set(keys_.angular_pose_error, pos_err.angle());

    // Write angular target speed as absolute
    io.set(keys_.angular_target_speed, etl::absolute(target_speed.angle()));

    DEBUG("OUTPUT: lin_err=%.2f ang_err=%.2f lin_tgt_spd=%.2f ang_tgt_spd=%.2f\n",
          static_cast<double>(pos_err.distance()), static_cast<double>(pos_err.angle()),
          static_cast<double>(etl::absolute(target_speed.distance())),
          static_cast<double>(etl::absolute(target_speed.angle())));

    // Write angular speed filter flag
    io.set(keys_.angular_speed_filter_flag, no_angular_limit_flag);

    // Write updated pose reached status
    target_pose_status_t reached = (current_state_ == PoseStraightFilterState::FINISHED)
                                       ? target_pose_status_t::reached
                                       : target_pose_status_t::moving;
    io.set(keys_.pose_reached, reached);

    // Write current state for feedforward profile triggering
    io.set(keys_.current_state, static_cast<int>(current_state_));

    // Write profile recompute signals (only if keys are configured)
    if (!keys_.linear_recompute_profile.empty()) {
        io.set(keys_.linear_recompute_profile, linear_recompute_profile);
        if (linear_recompute_profile) {
            DEBUG("Emitting linear_recompute_profile=true\n");
        }
    }
    if (!keys_.linear_invalidate_profile.empty()) {
        io.set(keys_.linear_invalidate_profile, linear_invalidate_profile);
        if (linear_invalidate_profile) {
            DEBUG("Emitting linear_invalidate_profile=true\n");
        }
    }
    if (!keys_.angular_recompute_profile.empty()) {
        io.set(keys_.angular_recompute_profile, angular_recompute_profile);
        if (angular_recompute_profile) {
            DEBUG("Emitting angular_recompute_profile=true\n");
        }
    }
    if (!keys_.angular_invalidate_profile.empty()) {
        io.set(keys_.angular_invalidate_profile, angular_invalidate_profile);
        if (angular_invalidate_profile) {
            DEBUG("Emitting angular_invalidate_profile=true\n");
        }
    }
}

} // namespace motion_control

} // namespace cogip
