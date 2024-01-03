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

double SpeedFilter::limit_speed_order(
    double speed_order,
    double target_speed,
    double min_speed,
    double max_speed,
    double max_acc
    )
{
    if (speed_order) {
        speed_order = std::max(fabs(speed_order), min_speed) * speed_order/fabs(speed_order);
    }

    // Limit target speed
    target_speed = std::min(target_speed, max_speed);

    // Limit speed command (maximum acceleration)
    double a = speed_order - previous_speed_order_;

    if (a > max_acc) {
        speed_order = previous_speed_order_ + max_acc;
    }

    if (a < -max_acc) {
        speed_order = previous_speed_order_ - max_acc;
    }

    // Limit speed command
    speed_order = std::min(speed_order, target_speed);
    speed_order = std::max(speed_order, -target_speed);

    previous_speed_order_ = speed_order;

    return speed_order;
}

void SpeedFilter::execute() {
    COGIP_DEBUG_COUT("Execute SpeedFilter");

    // Speed order
    double speed_order = this->inputs_[0];
    // Current speed
    double current_speed = this->inputs_[1];
    // Target speed
    double target_speed = this->inputs_[2];

    // Limit speed order
    speed_order = limit_speed_order(
        speed_order,
        target_speed,
        parameters_->min_speed(),
        parameters_->max_speed(),
        parameters_->max_acceleration()
        );

    // Store speed_error
    this->outputs_[0] = speed_order - current_speed;
};

} // namespace motion_control

} // namespace cogip
