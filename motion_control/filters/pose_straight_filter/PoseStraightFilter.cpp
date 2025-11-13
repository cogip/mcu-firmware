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
    DEBUG("Execute PoseStraightFilter");

    // Read current pose coordinates and orientation
    float current_pose_x = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_x)) {
        current_pose_x = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_pose_x.data(), current_pose_x);
    }
    float current_pose_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_y)) {
        current_pose_y = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_pose_y.data(), current_pose_y);
    }
    float current_pose_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_pose_O)) {
        current_pose_O = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_pose_O.data(), current_pose_O);
    }
    cogip_defs::Pose current_pose(current_pose_x, current_pose_y, current_pose_O);

    // Read target pose coordinates and orientation
    float target_pose_x = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose_x)) {
        target_pose_x = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_pose_x.data(), target_pose_x);
    }
    float target_pose_y = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose_y)) {
        target_pose_y = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_pose_y.data(), target_pose_y);
    }
    float target_pose_O = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_pose_O)) {
        target_pose_O = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_pose_O.data(), target_pose_O);
    }
    cogip_defs::Pose target_pose(target_pose_x, target_pose_y, target_pose_O);

    // Read current linear and angular speeds
    float curr_lin = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_linear_speed)) {
        curr_lin = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_linear_speed.data(), curr_lin);
    }
    float curr_ang = 0.0f;
    if (auto opt = io.get_as<float>(keys_.current_angular_speed)) {
        curr_ang = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.current_angular_speed.data(), curr_ang);
    }
    const cogip_defs::Polar current_speed(curr_lin, curr_ang);

    // Read target linear and angular speeds
    float target_pose_lin = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_linear_speed)) {
        target_pose_lin = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_linear_speed.data(), target_pose_lin);
    }
    float target_pose_ang = 0.0f;
    if (auto opt = io.get_as<float>(keys_.target_angular_speed)) {
        target_pose_ang = *opt;
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value %f",
                    keys_.target_angular_speed.data(), target_pose_ang);
    }
    cogip_defs::Polar target_speed(target_pose_lin, target_pose_ang);

    // Read motion direction mode
    cogip::path::motion_direction motion_dir = cogip::path::motion_direction::bidirectional;
    if (auto opt = io.get_as<int>(keys_.motion_direction)) {
        motion_dir = static_cast<cogip::path::motion_direction>(*opt);
    } else {
        LOG_WARNING("WARNING: %s is not available, using default value bidirectional",
                    keys_.motion_direction.data());
    }

    // Persist reverse permission when switching to MOVE_TO_POSITION state
    // and detect target changes to reset oscillation detection
    static bool force_can_reverse = false;
    static cogip_defs::Pose prev_target(INT32_MAX, INT32_MAX, INT32_MAX);
    static bool target_changed = false;

    if ((target_pose_x != prev_target.x()) || (target_pose_y != prev_target.y()) ||
        (target_pose_O != prev_target.O())) {
        force_can_reverse = false;
        target_changed = true;
        prev_target = target_pose;
    } else {
        target_changed = false;
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

    // Force can_reverse to true once motion has started
    if (force_can_reverse) {
        can_reverse = true;
        must_reverse = false;
    }

    // Compute pose error as polar difference
    cogip_defs::Polar pos_err = target_pose - current_pose;

    // Apply direction: reverse if must or if allowed and angle > 90°
    if (must_reverse) {
        DEBUG("Reverse error as must reverse\n");
        pos_err.reverse();
    }
    if (can_reverse && etl::absolute(pos_err.angle()) > 90.0f) {
        DEBUG("Reverse error as can reverse\n");
        pos_err.reverse();
    }

    bool no_angular_limit_flag = false;
    bool no_linear_limit_flag = false;

    const float linear_threshold = parameters_.linear_threshold();
    const float angular_threshold = parameters_.angular_threshold();
    const float linear_deceleration = parameters_.linear_deceleration();
    const float angular_deceleration = parameters_.angular_deceleration();
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
        DEBUG("ROTATE_TO_DIRECTION");
        if (absolute_angular_pose_error > angular_intermediate_threshold) {
            target_speed.set_distance(0.0f);
        } else {
            target_speed.set_distance(pos_err.distance() >= 0 ? 1.0f
                                                              : -1.0f); // forward motion sign
            force_can_reverse = true;
            current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
        }
        break;

    case PoseStraightFilterState::MOVE_TO_POSITION:
        DEBUG("MOVE_TO_POSITION");
        if (absolute_linear_pose_error <= linear_threshold) {
            current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
        }
        break;

    case PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE:
        DEBUG("ROTATE_TO_FINAL_ANGLE");
        {
            static float prev_angular_error = 0.0f;
            static bool first_entry = true;

            // Reset on target change
            if (target_changed) {
                first_entry = true;
                prev_angular_error = 0.0f;
            }

            if (!parameters_.bypass_final_orientation()) {
                pos_err.set_angle(limit_angle_deg(target_pose_O - current_pose_O));
            } else {
                pos_err.set_angle(0.0f);
            }
            target_speed.set_distance(0.0f);

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
        DEBUG("FINISHED");
        target_speed.set_distance(0.0f);
        target_speed.set_angle(0.0f);
        break;
    }

    // Apply deceleration rules if needed
    if (absolute_linear_pose_error <= ((curr_lin * curr_lin) / (2.0f * linear_deceleration))) {
        target_speed.set_distance(
            std::sqrt(2.0f * linear_deceleration * absolute_linear_pose_error));
    }
    if (absolute_angular_pose_error <= ((curr_ang * curr_ang) / (2.0f * angular_deceleration))) {
        target_speed.set_angle(
            std::sqrt(2.0f * angular_deceleration * absolute_angular_pose_error));
    }

    // Write linear pose error
    io.set(keys_.linear_pose_error, pos_err.distance());

    // Write linear target speed as absolute
    io.set(keys_.linear_target_speed, etl::absolute(target_speed.distance()));

    // Write linear speed filter flag
    io.set(keys_.linear_speed_filter_flag, static_cast<bool>(no_linear_limit_flag));

    // Write angular pose error
    io.set(keys_.angular_pose_error, pos_err.angle());

    // Write angular target speed as absolute
    io.set(keys_.angular_target_speed, etl::absolute(target_speed.angle()));

    // Write angular speed filter flag
    io.set(keys_.angular_speed_filter_flag, static_cast<bool>(no_angular_limit_flag));

    // Write updated pose reached status
    target_pose_status_t reached = (current_state_ == PoseStraightFilterState::FINISHED)
                                       ? target_pose_status_t::reached
                                       : target_pose_status_t::moving;
    io.set(keys_.pose_reached, reached);
}

} // namespace motion_control

} // namespace cogip
