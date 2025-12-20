// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    anti_blocking_controller Anti-Blocking Controller
/// @ingroup     motion_control_controllers
/// @brief       Controller for computing speed error and detecting blocking
/// @{
/// @file
/// @brief       Anti-Blocking controller class declaration
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "anti_blocking_controller/AntiBlockingControllerIOKeys.hpp"
#include "anti_blocking_controller/AntiBlockingControllerParameters.hpp"
#include "motion_control_common/Controller.hpp"

namespace cogip {

namespace motion_control {

/// @class AntiBlockingController
/// @brief Controller that computes speed error and detects blocking conditions.
///
/// This controller:
/// 1. Computes speed_error = speed_order - current_speed (for SpeedPID input)
/// 2. Detects blocking when speed is low despite a commanded speed
/// 3. Sets pose_reached to blocked when blocking is detected
class AntiBlockingController
    : public Controller<AntiBlockingControllerIOKeys, AntiBlockingControllerParameters>
{
  public:
    /// @brief Constructor.
    /// @param keys       IO key configuration
    /// @param parameters Anti-blocking parameters
    /// @param name       Optional instance name for identification
    AntiBlockingController(const AntiBlockingControllerIOKeys& keys,
                           const AntiBlockingControllerParameters& parameters,
                           etl::string_view name = "")
        : Controller<AntiBlockingControllerIOKeys, AntiBlockingControllerParameters>(
              keys, parameters, name),
          blocked_cycles_count_(0)
    {
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "AntiBlockingController";
    }

    /// @brief Execute the controller logic.
    /// @param io Reference to the shared ControllersIO storage.
    void execute(ControllersIO& io) override;

    /// @brief Reset the blocked cycles counter.
    void reset_blocked_cycles_count()
    {
        blocked_cycles_count_ = 0;
    }

    /// @brief Get current blocked cycles count.
    /// @return Current blocked cycles count.
    uint16_t blocked_cycles_count() const
    {
        return blocked_cycles_count_;
    }

  private:
    uint16_t blocked_cycles_count_; ///< Counter of consecutive blocked cycles
};

} // namespace motion_control

} // namespace cogip

/// @}
