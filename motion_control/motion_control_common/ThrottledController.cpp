// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "motion_control_common/ThrottledController.hpp"
#include "log.h"

#define ENABLE_DEBUG 0
#include <debug.h>
#include <inttypes.h>

namespace cogip {

namespace motion_control {

ThrottledController::ThrottledController(BaseController* wrapped_controller,
                                         uint16_t period_divider)
    : BaseController(), wrapped_controller_(wrapped_controller), period_divider_(period_divider),
      current_count_(period_divider)
{
    // Ensure period_divider is at least 1
    if (period_divider_ < 1) {
        LOG_ERROR("ThrottledController: period_divider must be >= 1, setting to 1\n");
        period_divider_ = 1;
    }
}

void ThrottledController::execute(ControllersIO& io)
{
    current_count_++;

    if (current_count_ >= period_divider_) {
        DEBUG("ThrottledController: executing wrapped controller (period=%" PRIu16 ")\n",
              period_divider_);
        if (wrapped_controller_) {
            wrapped_controller_->execute(io);
        }
        current_count_ = 0;
    } else {
        DEBUG("ThrottledController: skipping execution (count=%" PRIu16 "/%" PRIu16 ")\n",
              current_count_, period_divider_);
    }
}

void ThrottledController::set_period_divider(uint16_t period_divider)
{
    if (period_divider < 1) {
        LOG_ERROR("ThrottledController: period_divider must be >= 1, ignoring invalid value\n");
        return;
    }

    period_divider_ = period_divider;
    // Reset counter to avoid unexpected behavior
    current_count_ = 0;
}

} // namespace motion_control

} // namespace cogip
