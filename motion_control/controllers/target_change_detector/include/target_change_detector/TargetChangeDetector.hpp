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
#include "utils.hpp"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

/// @brief Generic Target Change Detector controller
///
/// Watches up to MAX_KEYS IO keys and triggers a new_target flag when any of
/// them changes. Each watched value is kept in its native variant type
/// (float / double / int / bool). Floats are compared with `areFloatsEqual`
/// and the configurable epsilon (default 1e-3), so small serialisation-noise
/// drifts (e.g. -400.00000000000006 vs -400.0) do not trigger a spurious
/// "new target". Integers and bools are compared exactly.
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
    /// @param keys Watched IO key names and output key.
    /// @param parameters Controller parameters (carries the float epsilon).
    /// @param name Optional controller instance name (for logging).
    explicit TargetChangeDetector(const TargetChangeDetectorIOKeys<MAX_KEYS>& keys,
                                  const TargetChangeDetectorParameters& parameters,
                                  etl::string_view name = "")
        : Controller<TargetChangeDetectorIOKeys<MAX_KEYS>, TargetChangeDetectorParameters>(
              keys, parameters, name),
          previous_values_(), first_run_(true)
    {
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

        // Read current values from all watched keys, keeping the native type.
        etl::array<ParamValue, MAX_KEYS> current_values = {};
        for (size_t i = 0; i < MAX_KEYS; i++) {
            if (this->keys_.watched_keys[i].empty()) {
                continue;
            }

            if (auto opt = io.get(this->keys_.watched_keys[i])) {
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
                if (!values_equivalent(current_values[i], previous_values_[i])) {
                    DEBUG("TargetChangeDetector: key %.*s changed\n",
                          static_cast<int>(this->keys_.watched_keys[i].size()),
                          this->keys_.watched_keys[i].data());
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
    /// @brief Compare two variant values, using an epsilon-based test for
    ///        floating-point types and exact equality for integer and bool.
    bool values_equivalent(const ParamValue& lhs, const ParamValue& rhs) const
    {
        if (lhs.index() != rhs.index()) {
            // Different active alternative -> treat as changed.
            return false;
        }

        const float epsilon = this->parameters_.epsilon();

        if (etl::holds_alternative<float>(lhs)) {
            return areFloatsEqual(etl::get<float>(lhs), etl::get<float>(rhs), epsilon);
        }
        if (etl::holds_alternative<double>(lhs)) {
            return areFloatsEqual(static_cast<float>(etl::get<double>(lhs)),
                                  static_cast<float>(etl::get<double>(rhs)), epsilon);
        }
        if (etl::holds_alternative<int>(lhs)) {
            return etl::get<int>(lhs) == etl::get<int>(rhs);
        }
        if (etl::holds_alternative<bool>(lhs)) {
            return etl::get<bool>(lhs) == etl::get<bool>(rhs);
        }
        // etl::string or any other alternative: fall back to exact compare.
        return lhs == rhs;
    }

    etl::array<ParamValue, MAX_KEYS> previous_values_; ///< Previously seen values
    bool first_run_;                                   ///< True on first execution
};

} // namespace motion_control

} // namespace cogip

/// @}
