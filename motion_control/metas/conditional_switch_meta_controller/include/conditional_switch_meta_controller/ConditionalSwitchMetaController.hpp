// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    conditional_switch_meta_controller
/// @{
/// @file
/// @brief      Conditional switch meta controller
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "motion_control_common/BaseController.hpp"
#include "motion_control_common/ControllersIO.hpp"
#include <etl/string.h>
#include <etl/string_view.h>

namespace cogip {

namespace motion_control {

/// @brief MetaController that switches between two controllers based on a boolean condition.
///
/// Reads a boolean value from ControllersIO and executes one of two controllers:
/// - controller_when_true if the condition key is true
/// - controller_when_false if the condition key is false
///
/// Typical use case: Switch between dominant controller (with strong gains for active motion)
/// and keeper controller (with weak gains for passive correction) based on profile invalidation.
///
/// Example: Angular axis
/// - During ROTATE states: invalidate_profile = false → use dominant controller (Kp=0.3)
/// - During MOVE state: invalidate_profile = true → use keeper controller (Kp=0.05)
class ConditionalSwitchMetaController : public BaseController
{
  public:
    /// Constructor
    /// @param condition_key Name of the boolean key in ControllersIO (e.g.
    /// "angular_invalidate_profile")
    /// @param controller_when_true Controller to execute when condition is true
    /// @param controller_when_false Controller to execute when condition is false
    ConditionalSwitchMetaController(const char* condition_key, BaseController* controller_when_true,
                                    BaseController* controller_when_false)
        : condition_key_(condition_key), controller_when_true_(controller_when_true),
          controller_when_false_(controller_when_false)
    {
    }

    /// Execute the appropriate controller based on the condition
    void execute(ControllersIO& io) override
    {
        // Read condition from IO (default to false if key not found)
        bool condition = false;
        // Create string_view from const char* at runtime
        // Use explicit length calculation since ETL might not have strlen constructor
        size_t len = 0;
        while (condition_key_[len] != '\0') {
            ++len;
        }
        etl::string_view key_view(condition_key_, len);
        if (auto opt = io.get_as<bool>(key_view)) {
            condition = *opt;
        }

        // Execute the selected controller
        if (condition) {
            controller_when_true_->execute(io);
        } else {
            controller_when_false_->execute(io);
        }
    }

    /// Get the type name of this controller
    const char* type_name() const override
    {
        return "ConditionalSwitchMetaController";
    }

    /// @brief Dump the controller hierarchy to stdout as ASCII tree
    /// @param indent Current indentation level
    /// @param is_last Whether this is the last child at current level
    /// @param prefix Prefix string for tree drawing
    /// @param counter Execution order counter (passed to children)
    void dump(int indent = 0, bool is_last = true, const char* prefix = "",
              int* counter = nullptr) const override
    {
        // Print tree branch for this meta controller
        if (indent > 0) {
            printf("%s%s", prefix, is_last ? "└── " : "├── ");
        }

        // Print type and condition key (no execution number for meta controllers)
        printf("%s [%s]\n", type_name(), condition_key_);

        // Build new prefix for children
        etl::string<128> new_prefix;
        new_prefix.append(prefix);
        new_prefix.append(is_last ? "    " : "│   ");

        // Dump "when false" branch first
        printf("%s├── [%s=false]:\n", new_prefix.c_str(), condition_key_);
        etl::string<128> false_prefix;
        false_prefix.append(new_prefix.c_str());
        false_prefix.append("│   ");
        if (controller_when_false_) {
            controller_when_false_->dump(indent + 2, true, false_prefix.c_str(), counter);
        }

        // Dump "when true" branch
        printf("%s└── [%s=true]:\n", new_prefix.c_str(), condition_key_);
        etl::string<128> true_prefix;
        true_prefix.append(new_prefix.c_str());
        true_prefix.append("    ");
        if (controller_when_true_) {
            controller_when_true_->dump(indent + 2, true, true_prefix.c_str(), counter);
        }
    }

  private:
    const char* condition_key_;             ///< Key name to read from ControllersIO
    BaseController* controller_when_true_;  ///< Controller executed when condition is true
    BaseController* controller_when_false_; ///< Controller executed when condition is false
};

} // namespace motion_control

} // namespace cogip

/// @}
