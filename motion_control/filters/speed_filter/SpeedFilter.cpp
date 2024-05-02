// RIOT includes
#include <ztimer.h>

// System includes
#include <cstdio>
#include <iostream>

// Project includes
#include "cogip_defs/Polar.hpp"
#include "etl/list.h"
#include "etl/vector.h"

#include "speed_filter/SpeedFilter.hpp"

namespace cogip {

namespace motion_control {

void SpeedFilter::limit_speed_order(
    double *speed_order,
    double target_speed,
    double current_speed,
    double min_speed,
    double max_speed,
    double max_acc
    )
{
    // Limit target speed
    target_speed = std::min(target_speed, max_speed);

    // Limit speed command (maximum acceleration)
    double a = *speed_order - current_speed;

    if (a > max_acc) {
        a =  max_acc;
    }

    if (a < -max_acc) {
        a = -max_acc;
    }

    *speed_order = current_speed + a;

    if (*speed_order < min_speed && *speed_order > -min_speed) {
        if (a > 0)
            *speed_order = min_speed;
        else if (a < 0)
            *speed_order = -min_speed;
    }

    // Limit speed command
    *speed_order = std::min(*speed_order, target_speed);
    *speed_order = std::max(*speed_order, -target_speed);
}

void SpeedFilter::execute() {
    COGIP_DEBUG_COUT("Execute SpeedFilter");

    // Speed order
    double speed_order = this->inputs_[0];
    // Current speed
    double current_speed = this->inputs_[1];
    // Target speed
    double target_speed = this->inputs_[2];
    // Do not filter speed order ?
    bool no_speed_filter = this->inputs_[3];
    // Pose reached
    target_pose_status_t pose_reached = (target_pose_status_t)this->inputs_[4];

    if (!no_speed_filter) {
        // Limit speed order
        limit_speed_order(
            &speed_order,
            target_speed,
            current_speed,
            parameters_->min_speed(),
            parameters_->max_speed(),
            parameters_->max_acceleration()
            );
    }

    // If anti blocking is activated
    if (parameters_->anti_blocking()) {
        // Check if the current speed of the robot is below a speed threshold, typically set low.
        // This is because if the robot is blocked by an obstacle, it may move slightly,
        // so a threshold must be considered as the speed won't be zero.
        // Additionally, ensure the robot isn't accelerating by comparing the current speed
        // with the speed order from the previous cycle. If the difference between these two
        // speeds exceeds a threshold, it indicates that the robot hasn't accelerated since its
        // previous cycle.
        if ((fabs(current_speed) < parameters_->anti_blocking_speed_threshold())
            && (fabs(previous_speed_order_ - current_speed) > parameters_->anti_blocking_error_threshold())) {
            // Increment a counter for cycles during which the robot is supposedly blocked.
            // This evaluation cannot be done within a single cycle, so the counter is incremented.
            anti_blocking_blocked_cycles_nb_++;
        }
        else {
            anti_blocking_blocked_cycles_nb_= 0;
        }

        // After a certain number of cycles, the robot is considered blocked.
        if (anti_blocking_blocked_cycles_nb_ > parameters_->anti_blocking_blocked_cycles_nb_threshold()) {
            std::cout << "BLOCKED" << std::endl;
            pose_reached = target_pose_status_t::blocked;
        }
    }

    previous_speed_order_ = speed_order;

    // Store speed_error
    this->outputs_[0] = speed_order - current_speed;
    // Pose reached
    this->outputs_[1] = pose_reached;
};

} // namespace motion_control

} // namespace cogip
