// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    telemetry_controller Telemetry controller
/// @{
/// @file
/// @brief      Telemetry controller
/// @author     Mathis LECRIVAIN <lecrivain.mathis@gmail.com>

#pragma once

// Project includes
#include "TelemetryControllerIOKeys.hpp"
#include "TelemetryControllerParameters.hpp"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

namespace cogip {

namespace motion_control {

/// @brief Telemetry controller.
///        Collects and transmits motion control telemetry data.
class TelemetryController
    : public Controller<TelemetryControllerIOKeys, TelemetryControllerParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       Reference to IO key names.
    /// @param parameters Reference to telemetry parameters.
    /// @param name       Optional instance name for identification.
    explicit TelemetryController(const TelemetryControllerIOKeys& keys,
                                 const TelemetryControllerParameters& parameters,
                                 etl::string_view name = "")
        : Controller<TelemetryControllerIOKeys, TelemetryControllerParameters>(keys, parameters,
                                                                               name)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "TelemetryController";
    }

    /// @brief Execute telemetry controller operations.
    void execute(ControllersIO& io) override;
};

} // namespace motion_control

} // namespace cogip

/// @}
