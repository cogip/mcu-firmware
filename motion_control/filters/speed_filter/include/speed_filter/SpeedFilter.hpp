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
class SpeedFilter : public Controller<5, 2, SpeedFilterParameters> {
public:
    /// @brief
    /// @param parameters
    explicit SpeedFilter(SpeedFilterParameters *parameters) : Controller(parameters),
        previous_speed_order_(0), anti_blocking_blocked_cycles_nb_(0) {};

    /// Limit acceleration and speed
    void execute() override;

    /// Get previous speed order
    /// return previous speed order
    float previous_speed_order() const { return previous_speed_order_; };

    /// Reset previous speed order
    void reset_previous_speed_order() { previous_speed_order_ = 0; };

    /// Reset anti blocking blocked cycle number
    void reset_anti_blocking_blocked_cycles_nb() { anti_blocking_blocked_cycles_nb_ = 0; };

protected:
    /// Previous cycle speed_order
    float previous_speed_order_;

    /// Anti blocking, number of blocked cycle
    uint32_t anti_blocking_blocked_cycles_nb_;

    void limit_speed_order(
        float *speed_order,
        float target_speed,
        float current_speed,
        float min_speed,
        float max_speed,
        float max_acc
    );

};

} // namespace motion_control

} // namespace cogip

/// @}
