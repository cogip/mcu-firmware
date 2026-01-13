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
    static cogip_defs::Pose start_pose(INT32_MAX, INT32_MAX, INT32_MAX);

    // Recompute profile signals - set to true only on specific state transitions
    bool linear_recompute_profile = false;
    bool angular_recompute_profile = false;

    // Static variable for FINISHED logging (reset on new target)
    static bool logged_finished = false;

    if ((target_pose_x != prev_target.x()) || (target_pose_y != prev_target.y()) ||
        (target_pose_O != prev_target.O())) {
        force_can_reverse = false;
        prev_target = target_pose;
        start_pose = current_pose;
        // Reset state machine to initial state on new target
        current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION;
        // Reset FINISHED logging flag
        logged_finished = false;
        // New external target -> recompute angular profile for initial rotation
        angular_recompute_profile = true;
        // Invalidate linear profile - will be recomputed when entering MOVE_TO_POSITION
        linear_recompute_profile = true;

        // Reset speed PIDs on new target
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
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

    DEBUG("pos_err AFTER reverse: dist=%.2f ang=%.2f\n", static_cast<double>(pos_err.distance()),
          static_cast<double>(pos_err.angle()));

    bool no_angular_limit_flag = false;
    bool no_linear_limit_flag = false;

    const float linear_threshold = parameters_.linear_threshold();

    const float absolute_linear_pose_error = etl::absolute(pos_err.distance());

    // State transitions - ensure we only move forward, never backward
    // Early transition from ROTATE_TO_DIRECTION if already close enough to target position
    if ((current_state_ == PoseStraightFilterState::ROTATE_TO_DIRECTION) &&
        (absolute_linear_pose_error <= linear_threshold)) {
        current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
        angular_recompute_profile = true;
        linear_recompute_profile = true;
    }

    // Delegate to state-specific methods
    if (current_state_ == PoseStraightFilterState::ROTATE_TO_DIRECTION) {
        rotate_to_direction(io, pos_err, current_pose, start_pose, force_can_reverse,
                            linear_recompute_profile, angular_recompute_profile);
    }

    if (current_state_ == PoseStraightFilterState::MOVE_TO_POSITION) {
        move_to_position(io, pos_err, current_pose, target_pose, linear_recompute_profile,
                         angular_recompute_profile);
    }

    if (current_state_ == PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE) {
        rotate_to_final_angle(io, pos_err, current_pose, target_pose, linear_recompute_profile,
                              angular_recompute_profile);
    }

    if (current_state_ == PoseStraightFilterState::FINISHED) {
        finished(pos_err, current_pose, target_pose, logged_finished);
    }

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
    if (!keys_.angular_recompute_profile.empty()) {
        io.set(keys_.angular_recompute_profile, angular_recompute_profile);
        if (angular_recompute_profile) {
            DEBUG("Emitting angular_recompute_profile=true\n");
        }
    }
}

void PoseStraightFilter::rotate_to_direction(ControllersIO& io, cogip_defs::Polar& pos_err,
                                             const cogip_defs::Pose& current_pose,
                                             const cogip_defs::Pose& start_pose,
                                             bool& force_can_reverse,
                                             bool& linear_recompute_profile,
                                             bool& angular_recompute_profile)
{
    DEBUG("ROTATE_TO_DIRECTION\n");

    const float angular_intermediate_threshold = parameters_.angular_intermediate_threshold();

    // Track previous angular error for continuity enforcement
    static float previous_angular_pose_error = 0.0f;

    // Reset continuity tracking when angular profile is recomputed
    if (angular_recompute_profile) {
        previous_angular_pose_error = 0.0f;
    }

    // Apply continuity enforcement to existing angular error (avoids 360° jumps at ±180° boundary)
    float raw_angular_error =
        enforce_angle_continuity_deg(pos_err.angle(), previous_angular_pose_error);
    pos_err.set_angle(raw_angular_error);

    const float absolute_angular_pose_error = etl::absolute(raw_angular_error);

    if (absolute_angular_pose_error > angular_intermediate_threshold) {
        // Anti-drift correction during initial rotation
        float pos_error_longitudinal = compute_longitudinal_error(
            current_pose.x(), current_pose.y(), current_pose.O(), start_pose.x(), start_pose.y());
        pos_err.set_distance(-pos_error_longitudinal);
    } else {
        force_can_reverse = true;
        current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
        // Reset speed PIDs on transition to MOVE_TO_POSITION
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
        // Transition to MOVE_TO_POSITION -> recompute profiles
        linear_recompute_profile = true;
        angular_recompute_profile = true;
        LOG_INFO("Transition to MOVE_TO_POSITION, (cur=%.2f + err=%.2f), "
                 "requesting angular and linear profile recompute\n",
                 static_cast<double>(current_pose.O()), static_cast<double>(pos_err.angle()));
    }
}

void PoseStraightFilter::move_to_position(ControllersIO& io, cogip_defs::Polar& pos_err,
                                          const cogip_defs::Pose& current_pose,
                                          cogip_defs::Pose& target_pose,
                                          bool& linear_recompute_profile,
                                          bool& angular_recompute_profile)
{
    DEBUG("MOVE_TO_POSITION\n");

    const float linear_threshold = parameters_.linear_threshold();

    // Bidirectional mode: dynamically adjust direction based on target position
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
        heading_error = raw_angle;
    } else {
        heading_error = limit_angle_deg(raw_angle + 180.0f);
    }
    pos_err.set_angle(heading_error);

    // Transition when close enough to target
    if (raw_distance <= linear_threshold) {
        current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
        // Reset speed PIDs on transition to ROTATE_TO_FINAL_ANGLE
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
        // Transition to ROTATE_TO_FINAL_ANGLE -> recompute profiles
        angular_recompute_profile = true;
        linear_recompute_profile = true;

        LOG_INFO("Transition to ROTATE_TO_FINAL_ANGLE, requesting linear and angular profiles "
                 "recompute\n");
    }
}

void PoseStraightFilter::rotate_to_final_angle(ControllersIO& io, cogip_defs::Polar& pos_err,
                                               const cogip_defs::Pose& current_pose,
                                               const cogip_defs::Pose& target_pose,
                                               bool& linear_recompute_profile,
                                               bool& angular_recompute_profile)
{
    DEBUG("ROTATE_TO_FINAL_ANGLE\n");

    const float angular_threshold = parameters_.angular_threshold();

    // Track previous angular error for continuity enforcement
    static float previous_angular_pose_error = 0.0f;

    // Reset continuity tracking when angular profile is recomputed
    if (angular_recompute_profile) {
        previous_angular_pose_error = 0.0f;
    }

    // Compute final angle error (to target orientation, not travel direction)
    float final_angle_error;
    if (!parameters_.bypass_final_orientation()) {
        // Calculate angular error with continuity enforcement (avoids 360° jumps at ±180° boundary)
        final_angle_error = angular_error_with_continuity_deg(target_pose.O(), current_pose.O(),
                                                              previous_angular_pose_error);
    } else {
        final_angle_error = 0.0f;
    }
    pos_err.set_angle(final_angle_error);

    // Anti-drift correction during final rotation
    float pos_error_longitudinal = compute_longitudinal_error(
        current_pose.x(), current_pose.y(), current_pose.O(), target_pose.x(), target_pose.y());
    pos_err.set_distance(-pos_error_longitudinal);

    if (etl::absolute(final_angle_error) > angular_threshold) {
        // Still rotating to final orientation with anti-drift correction active
        DEBUG("ROTATE_TO_FINAL_ANGLE: still rotating\n");
    } else {
        linear_recompute_profile = true;
        angular_recompute_profile = true;
        // Rotation complete - move to FINISHED
        current_state_ = PoseStraightFilterState::FINISHED;
        // Reset speed PIDs on transition to FINISHED
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
        LOG_INFO("ROTATE_TO_FINAL_ANGLE: rotation complete (ang=%.2f° <= threshold %.2f°)\n",
                 static_cast<double>(final_angle_error), static_cast<double>(angular_threshold));
    }
}

void PoseStraightFilter::finished(cogip_defs::Polar& pos_err, const cogip_defs::Pose& current_pose,
                                  cogip_defs::Pose& target_pose, bool& logged_finished)
{
    DEBUG("FINISHED\n");

    // Maintain position and orientation control in FINISHED state
    float pos_error_longitudinal = compute_longitudinal_error(
        current_pose.x(), current_pose.y(), current_pose.O(), target_pose.x(), target_pose.y());
    pos_err.set_distance(-pos_error_longitudinal);

    // Maintain final orientation
    float final_angle_error;
    if (!parameters_.bypass_final_orientation()) {
        final_angle_error = limit_angle_deg(target_pose.O() - current_pose.O());
    } else {
        final_angle_error = 0.0f;
    }
    pos_err.set_angle(final_angle_error);

    // Log final position error for debugging (only once per target)
    if (!logged_finished) {
        cogip_defs::Polar final_err = target_pose - current_pose;
        LOG_INFO("POSE_REACHED: linear_err=%.2f ang_err=%.2f final_ang_err=%.2f (cur: "
                 "%.1f,%.1f,%.1f -> tgt: "
                 "%.1f,%.1f,%.1f)\n",
                 static_cast<double>(final_err.distance()), static_cast<double>(final_err.angle()),
                 static_cast<double>(limit_angle_deg(target_pose.O() - current_pose.O())),
                 static_cast<double>(current_pose.x()), static_cast<double>(current_pose.y()),
                 static_cast<double>(current_pose.O()), static_cast<double>(target_pose.x()),
                 static_cast<double>(target_pose.y()), static_cast<double>(target_pose.O()));
        logged_finished = true;
    }
}

} // namespace motion_control

} // namespace cogip
