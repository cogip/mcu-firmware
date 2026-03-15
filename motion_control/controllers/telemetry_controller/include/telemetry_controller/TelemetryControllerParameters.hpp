// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    telemetry_controller Telemetry controller parameters
/// @{
/// @file
/// @brief      Telemetry controller parameters
/// @author     Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

namespace cogip {

namespace motion_control {

/// Telemetry controller parameters
struct TelemetryControllerParameters
{
    uint32_t loop_period_ms; ///< Motion control loop period (ms), for unit conversion
};

} // namespace motion_control

} // namespace cogip

/// @}
