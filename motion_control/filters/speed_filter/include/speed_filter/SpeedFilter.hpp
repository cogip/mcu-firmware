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
#include "SpeedFilterIOKeys.hpp"
#include "SpeedFilterParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {
namespace motion_control {

/// @brief Filter commanded speed according to acceleration and speed bounds.
///        Reads speed order, current speed, raw target speed, and disable flag,
///        then writes filtered speed error.
class SpeedFilter : public Controller<SpeedFilterIOKeys, SpeedFilterParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to a POD containing all input and output key names.
    /// @param parameters Reference to filtering parameters.
    /// @param name       Optional instance name for identification.
    explicit SpeedFilter(const SpeedFilterIOKeys& keys, const SpeedFilterParameters& parameters,
                         etl::string_view name = "")
        : Controller<SpeedFilterIOKeys, SpeedFilterParameters>(keys, parameters, name),
          previous_speed_order_(0.0f)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "SpeedFilter";
    }

    /// @brief Apply acceleration and speed limits, and compute speed error.
    /// @param io Shared ControllersIO containing inputs and receiving outputs.
    void execute(ControllersIO& io) override;

    /// @brief Get filtered speed from previous cycle.
    float previous_speed_order() const
    {
        return previous_speed_order_;
    }

    /// @brief Reset filtered speed to zero.
    void reset_previous_speed_order()
    {
        previous_speed_order_ = 0.0f;
    }

  protected:
    float previous_speed_order_; ///< filtered speed from previous cycle

    /// @brief Constrain commanded speed according to bounds and acceleration/deceleration.
    /// @param[in,out] speed_order  pointer to commanded speed; updated in place.
    /// @param[in]     raw_target   raw setpoint for maximum speed.
    /// @param[in]     min_speed    minimum non-zero speed threshold.
    /// @param[in]     max_speed    maximum absolute speed allowed.
    /// @param[in]     max_acc      maximum acceleration (speed increase) per cycle.
    /// @param[in]     max_dec      maximum deceleration (speed decrease) per cycle.
    void limit_speed_order(float* speed_order, float raw_target, float min_speed, float max_speed,
                           float max_acc, float max_dec);
};

} // namespace motion_control

} // namespace cogip

/// @}
