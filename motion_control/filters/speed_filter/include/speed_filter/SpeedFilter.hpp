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
#include "motion_control_common/ControllersIO.hpp"
#include "SpeedFilterParameters.hpp"
#include "SpeedFilterIOKeys.hpp"

namespace cogip {
namespace motion_control {

/// @brief Filter commanded speed according to acceleration and speed bounds,
///        and detect blocking.
///        Reads speed order, current speed, raw target speed, disable flag, and pose‐reached status,
///        then writes filtered speed error and updated pose‐reached status.
class SpeedFilter
    : public Controller<SpeedFilterIOKeys, SpeedFilterParameters>
{
public:
    /// @brief Constructor.
    /// @param keys       Reference to a POD containing all input and output key names.
    /// @param parameters Reference to filtering parameters.
    explicit SpeedFilter(
        const SpeedFilterIOKeys&         keys,
        const SpeedFilterParameters&     parameters
    )
        : Controller<SpeedFilterIOKeys, SpeedFilterParameters>(keys, parameters),
          previous_speed_order_(0.0f),
          anti_blocking_blocked_cycles_nb_(0)
    {
    }

    /// @brief Apply acceleration and speed limits, perform blocking detection,
    ///        and compute speed error and final pose‐reached status.
    /// @param io Shared ControllersIO containing inputs and receiving outputs.
    void execute(ControllersIO& io) override;

    /// @brief Get filtered speed from previous cycle.
    float previous_speed_order() const { return previous_speed_order_; }

    /// @brief Reset filtered speed to zero.
    void reset_previous_speed_order() { previous_speed_order_ = 0.0f; }

    /// @brief Reset blocked cycle counter to zero.
    void reset_anti_blocking_blocked_cycles_nb() { anti_blocking_blocked_cycles_nb_ = 0; }

protected:
    float previous_speed_order_;          ///< filtered speed from previous cycle
    uint32_t anti_blocking_blocked_cycles_nb_;  ///< count of consecutive blocked cycles

    /// @brief Constrain commanded speed according to bounds and acceleration.
    /// @param[in,out] speed_order  pointer to commanded speed; updated in place.
    /// @param[in]     raw_target   raw setpoint for maximum speed.
    /// @param[in]     min_speed    minimum non‐zero speed threshold.
    /// @param[in]     max_speed    maximum absolute speed allowed.
    /// @param[in]     max_acc      maximum change in speed per cycle.
    void limit_speed_order(
        float *speed_order,
        float raw_target,
        float min_speed,
        float max_speed,
        float max_acc
    );
};

} // namespace motion_control

} // namespace cogip

/// @}
