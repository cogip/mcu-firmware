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
    double current_pose_x = inputs_[input_index++];
    double current_pose_y = inputs_[input_index++];
    double current_pose_O = inputs_[input_index++];
    cogip_defs::Pose current_pose(
        current_pose_x,
        current_pose_y,
        current_pose_O
    );

    // Target pose
    double target_pose_x = this->inputs_[input_index++];
    double target_pose_y = this->inputs_[input_index++];
    double target_pose_O = this->inputs_[input_index++];
    cogip_defs::Pose target_pose(
        target_pose_x,
        target_pose_y,
        target_pose_O
    );

    // Current speed
    double current_linear_speed = this->inputs_[input_index++];
    double current_angular_speed = this->inputs_[input_index++];
    cogip_defs::Polar current_speed(
        current_linear_speed,
        current_angular_speed
    );

    // Target speed
    double target_linear_speed = this->inputs_[input_index++];
    double target_angular_speed = this->inputs_[input_index++];
    cogip_defs::Polar target_speed(
        target_linear_speed,
        target_angular_speed
    );

    // Allow moving reversely
    bool allow_reverse = (bool)inputs_[input_index++];

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

    // Each move is decomposed in three steps:
    //   1. The robot rotates on its center to take the direction of the pose to reach.
    //   2. The robot goes straight to that pose.
    //   3. Once arrived at this pose, it rotates again on its center to the wanted angle.
    // The angular treshold is used to check if a rotation of the robot on itself has to be done (step 1. and 3.).
    // The linear treshold is used to check if the robot is close to the pose to reach (step 2.).
    if (fabs(pos_err.distance()) > parameters_->linear_treshold()) {
        if (fabs(pos_err.angle()) > parameters_->angular_treshold()) {
            // So if the robot is far from the pose to reach and if the angle to reach this pose is too important,
            // first rotate on itself to take the direction of the destination.
            // Set the linear speed to 0 in such case.
            target_speed.set_distance(0);
        }
        else {
            // Once the angular direction is good, step 1. is completed, thus inform the platform through target pose status variable.
            pose_reached = target_pose_status_t::intermediate_reached;
        }
    }
    else {
        // If the linear error is below the linear treshold, the step 2. is completed.
        // Thus inform the platform through target pose status variable.
        pose_reached = target_pose_status_t::intermediate_reached;
        // Start step 3. to reach the final wanted orientation.
        pos_err.set_angle(limit_angle_deg(target_pose.O() - current_pose.O()));
        if (fabs(pos_err.angle()) < parameters_->angular_treshold()) {
            // Pose is finally reached, inform the platform through target pose status variable.
            pose_reached = target_pose_status_t::reached;
        }
    }

    // Linear pose error
    outputs_[0] = pos_err.distance();
    // Linear current speed
    outputs_[1] = current_speed.distance();
    // Linear target speed
    outputs_[2] = target_speed.distance();

    // Angular pose error
    outputs_[3] = pos_err.angle();
    // Angular current speed
    outputs_[4] = current_speed.angle();
    // Angular target speed
    outputs_[5] = target_speed.angle();

    // Pose reached
    outputs_[6] = (double)pose_reached;
};

} // namespace motion_control

} // namespace cogip
