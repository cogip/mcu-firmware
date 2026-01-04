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

    // Recompute profile signals - set to true only on specific state transitions
    bool linear_recompute_profile = false;
    bool linear_invalidate_profile = false;
    bool angular_recompute_profile = false;
    bool angular_invalidate_profile = false;

    // Static variable for FINISHED logging (reset on new target)
    static bool logged_finished = false;

    if ((target_pose_x != prev_target.x()) || (target_pose_y != prev_target.y()) ||
        (target_pose_O != prev_target.O())) {
        force_can_reverse = false;
        prev_target = target_pose;
        // Reset state machine to initial state on new target
        current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION;
        // Capture current position as reference for position holder during ROTATE_TO_DIRECTION
        rotate_to_direction_ref_x_ = current_pose_x;
        rotate_to_direction_ref_y_ = current_pose_y;
        // Reset FINISHED logging flag
        logged_finished = false;
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
    if (force_can_reverse) {
        can_reverse = true;
    }

    // Compute pose error as polar difference
    cogip_defs::Polar pos_err = target_pose - current_pose;

    DEBUG("pos_err BEFORE reverse: dist=%.2f ang=%.2f\n", static_cast<double>(pos_err.distance()),
          static_cast<double>(pos_err.angle()));
    DEBUG("motion_dir=%d can_reverse=%d must_reverse=%d\n", static_cast<int>(motion_dir),
          can_reverse, must_reverse);

    // Apply direction based on motion mode:
    // - For backward_only (must_reverse): always reverse to get negative distance (backward motion)
    //   The angle after reverse will be the optimal heading (away from target)
    // - For bidirectional (can_reverse): reverse only if |angle| > 90° (shorter rotation path)
    if (must_reverse || (can_reverse && (etl::absolute(pos_err.angle()) > 90.0f))) {
        // Always reverse for backward_only mode - ensures negative distance
        DEBUG("Reverse error as must_reverse (backward_only mode)\n");
        pos_err.reverse();
    }
    /*else if (can_reverse) {
        if (etl::absolute(pos_err.angle()) > 90.0f) {
            // Target is behind and we can choose -> reverse to go backward (shorter rotation)
            DEBUG("Reverse error as can reverse (angle > 90)\n");
            pos_err.reverse();
        }
    }*/

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
    case PoseStraightFilterState::ROTATE_TO_DIRECTION: {
        DEBUG("ROTATE_TO_DIRECTION\n");
        // Keep linear profile invalidated during initial rotation
        linear_invalidate_profile = true;
        
        // Position holder: maintain position where robot entered ROTATE_TO_DIRECTION state
        // Project position error onto robot's longitudinal axis for drift compensation
        float pos_error_longitudinal = compute_longitudinal_error(
            current_pose_x, current_pose_y, current_pose_O,
            rotate_to_direction_ref_x_, rotate_to_direction_ref_y_);
        pos_err.set_distance(-pos_error_longitudinal);
        
        // Apply position holder speed ratio to target_speed (0.2% of nominal speed)
        target_speed.set_distance(target_speed.distance() * parameters_.linear_pose_holder_speed_ratio());
        
        if (absolute_angular_pose_error > angular_intermediate_threshold) {
            // Still rotating to face target direction with position holding active
            DEBUG("ROTATE_TO_DIRECTION: still rotating (ang_err=%.2f pos_drift=%.2f)\n",
                  static_cast<double>(absolute_angular_pose_error),
                  static_cast<double>(pos_err.distance()));
        } else {
            force_can_reverse = true;
            current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
            // Reset speed PIDs on transition to MOVE_TO_POSITION
            io.set(keys_.linear_speed_pid_reset, true);
            io.set(keys_.angular_speed_pid_reset, true);
            // Transition to MOVE_TO_POSITION -> recompute linear profile and invalidate angular
            // profile
            linear_recompute_profile = true;
            angular_invalidate_profile = true;
            DEBUG("Transition to MOVE_TO_POSITION, (cur=%.2f + err=%.2f), "
                  "requesting linear profile recompute\n",
                  static_cast<double>(current_pose_O), static_cast<double>(pos_err.angle()));
        }
    } break;

    case PoseStraightFilterState::MOVE_TO_POSITION: {
        // Bidirectional mode: dynamically adjust direction based on target position
        // Recalculate raw polar error to determine if target is in front or behind
        cogip_defs::Polar raw_pos_err = target_pose - current_pose;
        float raw_distance = raw_pos_err.distance();
        float raw_angle = raw_pos_err.angle();

        // Target is in front if |angle| < 90°, behind otherwise
        bool target_is_in_front = (etl::absolute(raw_angle) < 90.0f);

        // Set linear error sign: positive if going forward, negative if going backward
        float linear_error = target_is_in_front ? raw_distance : -raw_distance;
        pos_err.set_distance(linear_error);

        // Set angular error to point towards (or away from) target
        float heading_error;
        if (target_is_in_front) {
            // Going forward: face the target
            heading_error = raw_angle;
        } else {
            // Going backward: face away from target
            heading_error = limit_angle_deg(raw_angle + 180.0f);
        }
        pos_err.set_angle(heading_error);

        // Keep angular profile invalidated during entire MOVE_TO_POSITION state
        angular_invalidate_profile = true;

        // Transition when close enough to target
        if (raw_distance <= linear_threshold) {
            current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
            // Reset speed PIDs on transition to ROTATE_TO_FINAL_ANGLE
            io.set(keys_.linear_speed_pid_reset, true);
            io.set(keys_.angular_speed_pid_reset, true);
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

    case PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE: {
        DEBUG("ROTATE_TO_FINAL_ANGLE\n");
        // Keep linear profile invalidated during final rotation
        linear_invalidate_profile = true;

        // Compute final angle error (to target orientation, not travel direction)
        float final_angle_error;
        if (!parameters_.bypass_final_orientation()) {
            final_angle_error = limit_angle_deg(target_pose_O - current_pose_O);
        } else {
            final_angle_error = 0.0f;
        }
        pos_err.set_angle(final_angle_error);

        // Anti-drift correction during final rotation
        // Project position error onto robot's longitudinal axis to compensate for rotation-induced
        // drift This keeps the robot at the target position without interfering with angular
        // convergence
        float pos_error_longitudinal = compute_longitudinal_error(
            current_pose_x, current_pose_y, current_pose_O, target_pose_x, target_pose_y);

        // Use projected error for weak linear correction (no target speed, only position holding)
        pos_err.set_distance(-pos_error_longitudinal);

        // Apply position holder speed ratio to target_speed (0.2% of nominal speed)
        target_speed.set_distance(target_speed.distance() * parameters_.linear_pose_holder_speed_ratio());

        DEBUG("ROTATE_TO_FINAL_ANGLE: final_ang=%.2f pos_drift=%.2f (threshold=%.2f)\n",
              static_cast<double>(final_angle_error), static_cast<double>(pos_error_longitudinal),
              static_cast<double>(angular_threshold));

        if (etl::absolute(final_angle_error) > angular_threshold) {
            // Still rotating to final orientation with anti-drift correction active
            DEBUG("ROTATE_TO_FINAL_ANGLE: still rotating\n");
        } else {
            // Rotation complete - move to FINISHED
            current_state_ = PoseStraightFilterState::FINISHED;
            // Reset speed PIDs on transition to FINISHED
            io.set(keys_.linear_speed_pid_reset, true);
            io.set(keys_.angular_speed_pid_reset, true);
            LOG_WARNING("ROTATE_TO_FINAL_ANGLE: rotation complete (ang=%.2f° <= threshold %.2f°)\n",
                        static_cast<double>(final_angle_error),
                        static_cast<double>(angular_threshold));
        }
    } break;

    case PoseStraightFilterState::FINISHED:
        DEBUG("FINISHED\n");
        // Keep profiles invalidated in FINISHED state
        linear_invalidate_profile = true;
        angular_invalidate_profile = true;

        // Maintain position and orientation control in FINISHED state
        // Project position error onto robot's longitudinal axis (same as ROTATE_TO_FINAL_ANGLE)
        {
            float pos_error_longitudinal = compute_longitudinal_error(
                current_pose_x, current_pose_y, current_pose_O, target_pose_x, target_pose_y);
            pos_err.set_distance(-pos_error_longitudinal);
        }

        // Maintain final orientation
        {
            float final_angle_error;
            if (!parameters_.bypass_final_orientation()) {
                final_angle_error = limit_angle_deg(target_pose_O - current_pose_O);
            } else {
                final_angle_error = 0.0f;
            }
            pos_err.set_angle(final_angle_error);
        }

        // Log final position error for debugging (only once per target)
        if (!logged_finished) {
            cogip_defs::Polar final_err = target_pose - current_pose;
            LOG_INFO("POSE_REACHED: linear_err=%.2f ang_err=%.2f final_ang_err=%.2f (cur: "
                     "%.1f,%.1f,%.1f -> tgt: "
                     "%.1f,%.1f,%.1f)\n",
                     static_cast<double>(final_err.distance()),
                     static_cast<double>(final_err.angle()),
                     static_cast<double>(limit_angle_deg(target_pose_O - current_pose_O)),
                     static_cast<double>(current_pose_x), static_cast<double>(current_pose_y),
                     static_cast<double>(current_pose_O), static_cast<double>(target_pose_x),
                     static_cast<double>(target_pose_y), static_cast<double>(target_pose_O));
            logged_finished = true;
        }
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
