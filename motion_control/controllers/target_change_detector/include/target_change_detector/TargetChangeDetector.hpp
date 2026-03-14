// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    target_change_detector Target Change Detector
/// @{
/// @file
/// @brief      Target Change Detector controller
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "TargetChangeDetectorIOKeys.hpp"
#include "TargetChangeDetectorParameters.hpp"
#include "log.h"
#include "motion_control_common/Controller.hpp"
#include "motion_control_common/ControllersIO.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

/// @brief Generic Target Change Detector controller
///
/// Watches up to MAX_KEYS IO keys and triggers a new_target flag when any of
/// them changes. Comparison is done by casting values to int (1-unit threshold).
///
/// @tparam MAX_KEYS Maximum number of IO keys to watch (default 4)
///
/// @code
/// // Pose mode: watch target_pose_x, target_pose_y, target_pose_O
/// TargetChangeDetectorIOKeys<3> keys = {
///     .watched_keys = {"target_pose_x", "target_pose_y", "target_pose_O"},
///     .new_target = "new_target",
/// };
///
/// // Speed tuning mode: watch target_speed and duration
/// TargetChangeDetectorIOKeys<2> keys = {
///     .watched_keys = {"linear_target_speed", "timeout_duration_period"},
///     .new_target = "recompute_profile",
/// };
/// @endcode
template <size_t MAX_KEYS = 4>
class TargetChangeDetector
    : public Controller<TargetChangeDetectorIOKeys<MAX_KEYS>, TargetChangeDetectorParameters>
{
  public:
    /// @brief Constructor
    explicit TargetChangeDetector(const TargetChangeDetectorIOKeys<MAX_KEYS>& keys,
                                  const TargetChangeDetectorParameters& parameters,
                                  etl::string_view name = "")
        : Controller<TargetChangeDetectorIOKeys<MAX_KEYS>, TargetChangeDetectorParameters>(
              keys, parameters, name),
          previous_values_(), first_run_(true)
    {
        previous_values_.fill(0);
    }

    /// @brief Get the type name of this controller
    const char* type_name() const override
    {
        return "TargetChangeDetector";
    }

    /// @brief Reset internal state
    /// Keep previous_values_ and first_run_ unchanged so that a reset
    /// does not spuriously trigger new_target on the next execute().
    /// Only a real change in watched keys should trigger new_target.
    void reset() override {}

    /// @brief Execute target change detection
    void execute(ControllersIO& io) override
    {
        DEBUG("Execute TargetChangeDetector\n");

        bool trigger_new_target = false;

        // Read current values from all watched keys
        etl::array<int, MAX_KEYS> current_values = {};
        for (size_t i = 0; i < MAX_KEYS; i++) {
            if (this->keys_.watched_keys[i].empty()) {
                continue;
            }

            // Try reading as float first, then as int
            if (auto opt = io.template get_as<float>(this->keys_.watched_keys[i])) {
                current_values[i] = static_cast<int>(*opt);
            } else if (auto opt = io.template get_as<int>(this->keys_.watched_keys[i])) {
                current_values[i] = *opt;
            } else {
                LOG_WARNING("TargetChangeDetector: key %.*s not available\n",
                            static_cast<int>(this->keys_.watched_keys[i].size()),
                            this->keys_.watched_keys[i].data());
                io.set(this->keys_.new_target, false);
                return;
            }
        }

        if (first_run_) {
            DEBUG("TargetChangeDetector: first run, triggering new target\n");
            trigger_new_target = true;
            previous_values_ = current_values;
            first_run_ = false;
        } else {
            for (size_t i = 0; i < MAX_KEYS; i++) {
                if (this->keys_.watched_keys[i].empty()) {
                    continue;
                }
                if (current_values[i] != previous_values_[i]) {
                    DEBUG("TargetChangeDetector: key %.*s changed (%d -> %d)\n",
                          static_cast<int>(this->keys_.watched_keys[i].size()),
                          this->keys_.watched_keys[i].data(), previous_values_[i],
                          current_values[i]);
                    trigger_new_target = true;
                    break;
                }
            }
            if (trigger_new_target) {
                previous_values_ = current_values;
            }
        }

        io.set(this->keys_.new_target, trigger_new_target);
    }

  private:
    etl::array<int, MAX_KEYS> previous_values_; ///< Previously stored values (cast to int)
    bool first_run_;                            ///< True on first execution
};

} // namespace motion_control

} // namespace cogip

/// @}
