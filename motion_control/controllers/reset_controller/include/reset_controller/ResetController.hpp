// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    reset_controller Reset controller
/// @{
/// @file
/// @brief      Controller that resets IO keys to specified initial values
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// ETL includes
#include "etl/string_view.h"
#include "etl/vector.h"

// Project includes
#include "motion_control_common/BaseController.hpp"
#include "motion_control_common/ControllersIO.hpp"

#include "log.h"

namespace cogip {

namespace motion_control {

/// @brief Maximum number of keys that can be reset by a single ResetController
static constexpr size_t RESET_CONTROLLER_MAX_KEYS = 8;

/// @brief Pair of key name and initial value to reset
struct ResetKeyValue
{
    etl::string_view key; ///< IO key name to reset
    float value;          ///< Initial value to set
};

/// @brief Controller that resets specified IO keys to their initial values.
///        Should be placed at the beginning of a controller chain to ensure
///        clean state before processing.
class ResetController : public BaseController
{
  public:
    /// @brief Constructor with initializer list of key-value pairs.
    /// @param keys_values Initializer list of {key, value} pairs to reset
    /// @param name        Optional instance name for identification
    ResetController(std::initializer_list<ResetKeyValue> keys_values, etl::string_view name = "")
        : BaseController(name), keys_values_()
    {
        for (const auto& kv : keys_values) {
            if (keys_values_.size() < RESET_CONTROLLER_MAX_KEYS) {
                keys_values_.push_back(kv);
            }
        }
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "ResetController";
    }

    /// @brief Reset all configured IO keys to their initial values.
    /// @param io Shared ControllersIO to reset values in
    void execute(ControllersIO& io) override
    {
        LOG_INFO("ResetController: executing\n");
        for (const auto& kv : keys_values_) {
            io.set(kv.key, kv.value);
        }
    }

  private:
    etl::vector<ResetKeyValue, RESET_CONTROLLER_MAX_KEYS>
        keys_values_; ///< Keys and values to reset
};

} // namespace motion_control

} // namespace cogip

/// @}
