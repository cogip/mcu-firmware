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

    previous_speed_order_ = *speed_order;
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

    // Store speed_error
    this->outputs_[0] = speed_order - current_speed;
};

} // namespace motion_control

} // namespace cogip
