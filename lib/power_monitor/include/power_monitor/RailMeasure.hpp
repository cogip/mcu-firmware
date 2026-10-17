// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_power_monitor
/// @{
/// @file
/// @brief       Power rail measure definition
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

#include "PB_RailMeasure.hpp"

namespace cogip {
namespace power_monitor {

/// @brief Voltage and current measured on a power rail
struct RailMeasure
{
    uint32_t voltage_mv; ///< Rail voltage in millivolts
    uint32_t current_ma; ///< Rail current in milliamps

    /// @brief Copy data to Protobuf message
    /// @param[out] pb_rail_measure Protobuf message to fill
    void pb_copy(PB_RailMeasure& pb_rail_measure) const
    {
        pb_rail_measure.set_voltage_mv(voltage_mv);
        pb_rail_measure.set_current_ma(current_ma);
    }
};

} // namespace power_monitor
} // namespace cogip

/// @}
