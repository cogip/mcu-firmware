// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "cogip_defs/Pose.hpp"
#include "cogip_defs/Polar.hpp"
#include "etl/list.h"
#include "etl/vector.h"

#include "pose_straight_filter/PoseStraightFilter.hpp"

namespace cogip {

namespace motion_control {

void PoseStraightFilter::execute() {
    COGIP_DEBUG_COUT("Execute PoseStraightFilter");

    size_t input_index = 0;

    // Current pose
    float current_pose_x = inputs_[input_index++];
    float current_pose_y = inputs_[input_index++];
    float current_pose_O = inputs_[input_index++];
    cogip_defs::Pose current_pose(
        current_pose_x,
        current_pose_y,
        current_pose_O
    );

    // Target pose
    float target_pose_x = this->inputs_[input_index++];
    float target_pose_y = this->inputs_[input_index++];
    float target_pose_O = this->inputs_[input_index++];
    cogip_defs::Pose target_pose(
        target_pose_x,
        target_pose_y,
        target_pose_O
    );

    // Current speed
    float current_linear_speed = this->inputs_[input_index++];
    float current_angular_speed = this->inputs_[input_index++];
    cogip_defs::Polar current_speed(
        current_linear_speed,
        current_angular_speed
    );

    // Target speed
    float target_linear_speed = this->inputs_[input_index++];
    float target_angular_speed = this->inputs_[input_index++];
    cogip_defs::Polar target_speed(
        target_linear_speed,
        target_angular_speed
    );

    // Allow moving reversely
    bool allow_reverse = (bool)inputs_[input_index++];

    // Force allow-reverse once step 2 is done
    static bool force_allow_reverse = false;

    // Keep trace of previous target pose
    static cogip_defs::Pose previous_target_pose(
        INT32_MAX,
        INT32_MAX,
        INT32_MAX
    );

    if (
        (target_pose.x() != previous_target_pose.x())
        || (target_pose.y() != previous_target_pose.y())
        || (target_pose.O() != previous_target_pose.O())) {
        force_allow_reverse = false;

        previous_target_pose = target_pose;
    }

    if (force_allow_reverse) {
        allow_reverse = true;
    }

    if (!is_index_valid(input_index)) {
        return;
    }

    // Position reached flag
    target_pose_status_t pose_reached = target_pose_status_t::moving;

    // compute position error
    cogip_defs::Polar pos_err = target_pose - current_pose;

    // If the robot is allowed to go backward, reverse the error if it shorten the move.
    if (allow_reverse && (fabs(pos_err.angle()) > 90)) {
        pos_err.reverse();
    }

    bool no_angular_speed_limit = false;
    bool no_linear_speed_limit = false;

    // Each move is decomposed in three steps:
    //   1. The robot rotates on its center to take the direction of the pose to reach.
    //   2. The robot goes straight to that pose.
    //   3. Once arrived at this pose, it rotates again on its center to the wanted angle.
    // The angular threshold is used to check if a rotation of the robot on itself has to be done.
    // The linear threshold is used to check if the robot is close to the pose to reach.
    if ((this->current_state_ == PoseStraightFilterState::ROTATE_TO_DIRECTION)
        && (fabs((pos_err.distance())) <= parameters_->linear_threshold())) {
        // If already on target pose, directly take orientation.
        this->current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
    }
    switch (this->current_state_) {
        case PoseStraightFilterState::ROTATE_TO_DIRECTION:
            if (fabs(pos_err.angle()) > parameters_->angular_intermediate_threshold()) {
                // Rotate towards the direction of the target pose
                // Set linear speed to 0
                target_speed.set_distance(0);
            } else {
                // Angular direction correct, move to the next step
                pose_reached = target_pose_status_t::intermediate_reached;
                force_allow_reverse = true;
                this->current_state_ = PoseStraightFilterState::MOVE_TO_POSITION;
            }
            break;

        case PoseStraightFilterState::MOVE_TO_POSITION:
            // Move towards the target pose
            // Angular speed and linear threshold unrestricted
            if (fabs(pos_err.distance()) <= parameters_->linear_threshold()) {
                // Reached intermediate pose, move to the final step
                pose_reached = target_pose_status_t::intermediate_reached;
                this->current_state_ = PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE;
            }
            break;

        case PoseStraightFilterState::ROTATE_TO_FINAL_ANGLE:
            // Check if final orientation should be bypassed
            if (!parameters_->bypass_final_orientation()) {
                pos_err.set_angle(limit_angle_deg(target_pose.O() - current_pose.O()));
            }
            else {
                pos_err.set_angle(0);
            }
            // Force linear speed to 0
            target_speed.set_distance(0);

            // Rotate towards the final orientation
            if (fabs(pos_err.angle()) <= parameters_->angular_threshold()) {
                // Reached final pose
                pose_reached = target_pose_status_t::reached;
                this->current_state_ = PoseStraightFilterState::FINISHED;
            }
            break;

        case PoseStraightFilterState::FINISHED:
            // All steps completed, nothing to do
            target_speed.set_distance(0);
            target_speed.set_angle(0);
            break;
    }

    if (fabs(pos_err.distance()) <= ((current_speed.distance() * current_speed.distance()) / (2 * parameters_->linear_deceleration()))) {
        target_speed.set_distance(sqrt(2 * parameters_->linear_deceleration() * fabs(pos_err.distance())));
    }

    if (fabs(pos_err.angle()) <= ((current_speed.angle() * current_speed.angle()) / (2 * parameters_->angular_deceleration()))) {
        target_speed.set_angle(sqrt(2 * parameters_->angular_deceleration() * fabs(pos_err.angle())));
    }

    // Linear pose error
    outputs_[0] = pos_err.distance();
    // Linear current speed
    outputs_[1] = current_speed.distance();
    // Linear target speed
    outputs_[2] = fabs(target_speed.distance());
    // Should linear speed be filtered?
    outputs_[3] = (float)no_linear_speed_limit;

    // Angular pose error
    outputs_[4] = pos_err.angle();
    // Angular current speed
    outputs_[5] = current_speed.angle();
    // Angular target speed
    outputs_[6] = fabs(target_speed.angle());
    // Should angular speed be filtered?
    outputs_[7] = (float)no_angular_speed_limit;

    // Pose reached
    outputs_[8] = (float)pose_reached;
};

} // namespace motion_control

} // namespace cogip
