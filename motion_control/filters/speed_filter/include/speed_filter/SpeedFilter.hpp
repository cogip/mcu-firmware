// Copyright (C) 2023 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    speed_filter Speed filter
/// @{
/// @file
/// @brief      Filter maximum speed and acceleration
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "motion_control_common/Controller.hpp"
#include "SpeedFilterParameters.hpp"

namespace cogip {

namespace motion_control {

/// Filter maximum speed and acceleration
/// Input 0:    speed order
/// Input 1:    current speed
/// Input 2:    target speed
/// Output 0:   filtered speed
class SpeedFilter : public Controller<4, 1, SpeedFilterParameters> {
public:
    /// @brief
    /// @param parameters
    explicit SpeedFilter(SpeedFilterParameters *parameters) : Controller(parameters), previous_speed_order_(0) {};

    /// Limit acceleration and speed
    void execute() override;

    /// Get previous speed order
    /// return previous speed order
    double previous_speed_order() const { return previous_speed_order_; };

protected:
    /// Previous cycle speed_order
    double previous_speed_order_;

    double limit_speed_order(
        double speed_order,
        double target_speed,
        double current_speed,
        double min_speed,
        double max_speed,
        double max_acc
    );

};

} // namespace motion_control

} // namespace cogip

/// @}
