// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    target_change_detector
/// @{
/// @file
/// @brief      Target Change Detector implementation
/// @author     Gilles DOFFE <g.doffe@gmail.com>

// Project includes
#include "target_change_detector/TargetChangeDetector.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>

namespace cogip {

namespace motion_control {

TargetChangeDetector::TargetChangeDetector(const TargetChangeDetectorIOKeys& keys,
                                           const TargetChangeDetectorParameters& parameters)
    : Controller<TargetChangeDetectorIOKeys, TargetChangeDetectorParameters>(keys, parameters),
      previous_target_x_(0.0), previous_target_y_(0.0), previous_target_O_(0.0),
      previous_state_(-1), first_run_(true)
{
}

void TargetChangeDetector::execute(ControllersIO& io)
{
    DEBUG("Execute TargetChangeDetector\n");

    bool trigger_new_target = false;

    // Check for state transition mode (if current_state key is configured)
    bool state_transition_mode = !keys_.current_state.empty();

    if (state_transition_mode) {
        // State transition mode: trigger when entering the specified state
        int current_state = -1;
        if (auto opt = io.get_as<int>(keys_.current_state)) {
            current_state = *opt;
        } else {
            LOG_WARNING("TargetChangeDetector: current_state not available\n");
            io.set(keys_.new_target, false);
            return;
        }

        // Trigger on transition TO the trigger_state
        if (current_state == keys_.trigger_state && previous_state_ != keys_.trigger_state) {
            DEBUG("TargetChangeDetector: state transition to %d, triggering new target\n",
                  keys_.trigger_state);
            trigger_new_target = true;
        }

        previous_state_ = current_state;
    } else {
        // Target change mode: trigger when target coordinates change
        float current_target_x = 0.0f;
        bool has_target_x = !keys_.target_x.empty();
        if (has_target_x) {
            if (auto opt = io.get_as<float>(keys_.target_x)) {
                current_target_x = *opt;
            } else {
                LOG_WARNING("TargetChangeDetector: target_x not available\n");
                io.set(keys_.new_target, false);
                return;
            }
        }

        float current_target_y = 0.0f;
        bool has_target_y = !keys_.target_y.empty();
        if (has_target_y) {
            if (auto opt = io.get_as<float>(keys_.target_y)) {
                current_target_y = *opt;
            } else {
                LOG_WARNING("TargetChangeDetector: target_y not available\n");
                io.set(keys_.new_target, false);
                return;
            }
        }

        float current_target_O = 0.0f;
        bool has_target_O = !keys_.target_O.empty();
        if (has_target_O) {
            if (auto opt = io.get_as<float>(keys_.target_O)) {
                current_target_O = *opt;
            } else {
                LOG_WARNING("TargetChangeDetector: target_O not available\n");
                io.set(keys_.new_target, false);
                return;
            }
        }

        // On first run, always trigger new target
        if (first_run_) {
            DEBUG("TargetChangeDetector: first run, triggering new target\n");
            trigger_new_target = true;
            previous_target_x_ = current_target_x;
            previous_target_y_ = current_target_y;
            previous_target_O_ = current_target_O;
            first_run_ = false;
        } else {
            // Check if target has changed (cast to int for 1-unit threshold comparison)
            bool target_changed =
                (has_target_x &&
                 (static_cast<int>(current_target_x) != static_cast<int>(previous_target_x_))) ||
                (has_target_y &&
                 (static_cast<int>(current_target_y) != static_cast<int>(previous_target_y_))) ||
                (has_target_O &&
                 (static_cast<int>(current_target_O) != static_cast<int>(previous_target_O_)));

            if (target_changed) {
                DEBUG("TargetChangeDetector: target changed\n");
                trigger_new_target = true;
                previous_target_x_ = current_target_x;
                previous_target_y_ = current_target_y;
                previous_target_O_ = current_target_O;
            }
        }
    }

    io.set(keys_.new_target, trigger_new_target);
}

} // namespace motion_control

} // namespace cogip

/// @}
