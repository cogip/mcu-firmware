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

    // Read bypass_final_orientation from IO (set by PathManagerFilter)
    // Fall back to parameters if key is empty or not set in IO
    if (!keys_.bypass_final_orientation.empty()) {
        if (auto opt = io.get_as<bool>(keys_.bypass_final_orientation)) {
            bypass_final_orientation_ = *opt;
        }
        // If not in IO yet, keep previous value (default false)
    } else {
        // If no IO key configured, fall back to parameters
        bypass_final_orientation_ = parameters_.bypass_final_orientation();
    }

    // Recompute profile signals - set to true only on specific state transitions
    bool linear_recompute_profile = false;
    bool angular_recompute_profile = false;

    if ((target_pose_x != prev_target_.x()) || (target_pose_y != prev_target_.y()) ||
        (target_pose_O != prev_target_.O())) {
        prev_target_ = target_pose;
        start_pose_ = current_pose;
        // Reset state machine to initial state on new target
        current_state_ = PoseStraightFilterState::ROTATE_TO_DIRECTION;
        // Reset FINISHED logging flag
        logged_finished_ = false;
        // Reset angular error tracking
        prev_angular_error_rotate_ = 0.0f;
        prev_angular_error_final_ = 0.0f;
        // New external target -> recompute angular profile for initial rotation
        angular_recompute_profile = true;
        // Invalidate linear profile - will be recomputed when entering MOVE_TO_POSITION
        linear_recompute_profile = true;

        // Reset speed PIDs on new target
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
        io.set(keys_.linear_pose_pid_reset, true);
        io.set(keys_.angular_pose_pid_reset, true);

        // Lock reverse decision for this waypoint based on motion direction and initial angle
        // This prevents instability when angle is close to 90°
        cogip_defs::Polar initial_err = target_pose - current_pose;
        switch (motion_dir) {
        case cogip::path::motion_direction::forward_only:
            // Never reverse
            locked_reverse_ = false;
            break;
        case cogip::path::motion_direction::backward_only:
            // Always reverse
            locked_reverse_ = true;
            break;
        case cogip::path::motion_direction::bidirectional:
        default:
            // Optimal choice: reverse if |angle| > 90° (decided once, locked for this waypoint)
            locked_reverse_ = (etl::absolute(initial_err.angle()) > 90.0f);
            break;
        }
        DEBUG("New waypoint: locked_reverse_=%d (initial_angle=%.2f, motion_dir=%d)\n",
              locked_reverse_, static_cast<double>(initial_err.angle()),
              static_cast<int>(motion_dir));
    }

    // Compute pose error as polar difference
    cogip_defs::Polar pos_err = target_pose - current_pose;

    DEBUG("pos_err BEFORE reverse: dist=%.2f ang=%.2f\n", static_cast<double>(pos_err.distance()),
          static_cast<double>(pos_err.angle()));
    DEBUG("locked_reverse_=%d\n", locked_reverse_);

    // Apply locked reverse decision (stable for the entire waypoint)
    if (locked_reverse_) {
        DEBUG("Applying reverse (locked decision)\n");
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
        DEBUG("Early transition to ROTATE_TO_FINAL_ANGLE (already at position, linear_err=%.2f "
              "<= threshold %.2f)\n",
              static_cast<double>(absolute_linear_pose_error),
              static_cast<double>(linear_threshold));
    }

    if (current_state_ == PoseStraightFilterState::ROTATE_TO_DIRECTION) {
        rotate_to_direction(io, pos_err, current_pose, start_pose_, linear_recompute_profile,
                            angular_recompute_profile);
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
        finished(pos_err, current_pose, target_pose);
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
    target_pose_status_t reached;
    if (current_state_ == PoseStraightFilterState::FINISHED) {
        reached = target_pose_status_t::reached;
    } else {
        reached = target_pose_status_t::moving;
    }
    io.set(keys_.pose_reached, reached);

    // Write current state for tracker profile triggering
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
                                             bool& linear_recompute_profile,
                                             bool& angular_recompute_profile)
{
    DEBUG("ROTATE_TO_DIRECTION\n");

    const float angular_intermediate_threshold = parameters_.angular_intermediate_threshold();

    // Apply angle normalization (continuity enforcement or simple limit)
    float raw_angular_error;
    if (parameters_.use_angle_continuity()) {
        // Reset continuity tracking when angular profile is recomputed
        if (angular_recompute_profile) {
            prev_angular_error_rotate_ = pos_err.angle();
        }
        // Apply continuity enforcement (avoids 360° jumps at ±180° boundary for ProfileTracker)
        raw_angular_error =
            enforce_angle_continuity_deg(pos_err.angle(), prev_angular_error_rotate_);
    } else {
        // Simple limit to [-180, 180] (for direct PID control)
        raw_angular_error = limit_angle_deg(pos_err.angle());
    }
    pos_err.set_angle(raw_angular_error);

    const float absolute_angular_pose_error = etl::absolute(raw_angular_error);

    if (absolute_angular_pose_error > angular_intermediate_threshold) {
        // Anti-drift correction during initial rotation
        float pos_error_longitudinal = compute_longitudinal_error(
            current_pose.x(), current_pose.y(), current_pose.O(), start_pose.x(), start_pose.y());
        pos_err.set_distance(-pos_error_longitudinal);
    } else {
        current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
        // Reset speed PIDs on transition to MOVE_TO_POSITION
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
        io.set(keys_.linear_pose_pid_reset, true);
        io.set(keys_.angular_pose_pid_reset, true);
        // Transition to MOVE_TO_POSITION -> recompute profiles
        linear_recompute_profile = true;
        angular_recompute_profile = true;
        DEBUG("Transition to MOVE_TO_POSITION, (cur=%.2f + err=%.2f), "
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
        io.set(keys_.linear_pose_pid_reset, true);
        io.set(keys_.angular_pose_pid_reset, true);
        // Transition to ROTATE_TO_FINAL_ANGLE -> recompute profiles
        angular_recompute_profile = true;
        linear_recompute_profile = true;

        DEBUG("Transition to ROTATE_TO_FINAL_ANGLE, requesting linear and angular profiles "
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

    // Compute final angle error (to target orientation, not travel direction)
    float final_angle_error;
    if (!bypass_final_orientation_) {
        float raw_error = limit_angle_deg(target_pose.O() - current_pose.O());
        if (parameters_.use_angle_continuity()) {
            // Use continuity enforcement to avoid 360° jumps (for ProfileTracker)
            if (angular_recompute_profile) {
                prev_angular_error_final_ = raw_error;
            }
            final_angle_error = enforce_angle_continuity_deg(raw_error, prev_angular_error_final_);
        } else {
            // Simple limit (for direct PID control)
            final_angle_error = raw_error;
        }
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
    } else {
        linear_recompute_profile = true;
        angular_recompute_profile = true;
        // Rotation complete - move to FINISHED
        current_state_ = PoseStraightFilterState::FINISHED;
        // Reset speed PIDs on transition to FINISHED
        io.set(keys_.linear_speed_pid_reset, true);
        io.set(keys_.angular_speed_pid_reset, true);
        io.set(keys_.linear_pose_pid_reset, true);
        io.set(keys_.angular_pose_pid_reset, true);
    }
}

void PoseStraightFilter::finished(cogip_defs::Polar& pos_err, const cogip_defs::Pose& current_pose,
                                  cogip_defs::Pose& target_pose)
{
    DEBUG("FINISHED\n");

    // Maintain position and orientation control in FINISHED state
    float pos_error_longitudinal = compute_longitudinal_error(
        current_pose.x(), current_pose.y(), current_pose.O(), target_pose.x(), target_pose.y());
    pos_err.set_distance(-pos_error_longitudinal);

    // Maintain final orientation
    float final_angle_error;
    if (!bypass_final_orientation_) {
        final_angle_error = limit_angle_deg(target_pose.O() - current_pose.O());
    } else {
        final_angle_error = 0.0f;
    }
    pos_err.set_angle(final_angle_error);

    // Log final position error for debugging (only once per target)
    if (!logged_finished_) {
        cogip_defs::Polar final_err = target_pose - current_pose;
        DEBUG("POSE_REACHED: linear_err=%.2f ang_err=%.2f final_ang_err=%.2f (cur: "
              "%.1f,%.1f,%.1f -> tgt: "
              "%.1f,%.1f,%.1f)\n",
              static_cast<double>(final_err.distance()), static_cast<double>(final_err.angle()),
              static_cast<double>(limit_angle_deg(target_pose.O() - current_pose.O())),
              static_cast<double>(current_pose.x()), static_cast<double>(current_pose.y()),
              static_cast<double>(current_pose.O()), static_cast<double>(target_pose.x()),
              static_cast<double>(target_pose.y()), static_cast<double>(target_pose.O()));
        logged_finished_ = true;
    }
}

} // namespace motion_control

} // namespace cogip
