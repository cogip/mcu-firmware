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

    // Transition between straight move and final orientation
    static bool transition_straight_to_rotate = false;

    // position correction
    if (fabs(pos_err.distance()) > parameters_->linear_treshold()) {
        // Transition between orientation to the point and straight move
        static bool transition_rotate_to_straight = false;

        // go reverse ? */
        if (allow_reverse && fabs(pos_err.angle()) > 90) {
            pos_err.reverse();
        }

        // if target point direction angle is too important, bot rotates on its starting point
        if (fabs(pos_err.angle()) > parameters_->angular_treshold()) {
            transition_rotate_to_straight = true;
            pos_err.set_distance(0);
        }
        else if (transition_rotate_to_straight) {
            pose_reached = target_pose_status_t::intermediate_reached;
            transition_rotate_to_straight = false;
            transition_straight_to_rotate = true;
        }
    }
    else {
        if (transition_straight_to_rotate) {
            pose_reached = target_pose_status_t::intermediate_reached;
            transition_straight_to_rotate = false;
        }

        // orientation correction only (position is reached)
        pos_err.set_distance(0);

        // final orientation error
        pos_err.set_angle(limit_angle_deg(target_pose.O() - current_pose.O()));

        // orientation is reached
        if (fabs(pos_err.angle()) < parameters_->angular_treshold()) {
            pose_reached = target_pose_status_t::reached;
            pos_err.set_angle(0);
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
