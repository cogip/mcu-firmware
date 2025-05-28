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

    /// @brief Applies acceleration and speed bounds to the commanded speed.
    /// @details
    ///   1. Clamps the target_speed to the range [-max_speed, max_speed].
    ///   2. Computes the requested acceleration as (*speed_order) minus previous_speed_order_
    ///      and limits it to the range [-max_acc, max_acc].
    ///   3. Updates *speed_order to previous_speed_order_ plus the limited acceleration.
    ///   4. Applies a minimum speed threshold: if the absolute value of *speed_order is
    ///      less than min_speed, snaps it to +min_speed or -min_speed depending on the sign
    ///      of the acceleration.
    ///   5. Finally clamps *speed_order to the range [-target_speed, target_speed].
    ///
    /// @param[in,out] speed_order  Pointer to the current speed command; updated in place.
    /// @param[in]     target_speed Desired speed setpoint (will be clamped to ±max_speed).
    /// @param[in]     min_speed    Minimum non-zero speed magnitude (deadband threshold).
    /// @param[in]     max_speed    Maximum allowable speed magnitude.
    /// @param[in]     max_acc      Maximum allowable change in speed per call (acceleration limit).
    void limit_speed_order(
        float *speed_order,
        float target_speed,
        float min_speed,
        float max_speed,
        float max_acc
    );
};

} // namespace motion_control

} // namespace cogip

/// @}
