// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Throttled controller wrapper to execute at reduced frequency
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

// Project includes
#include "BaseController.hpp"

namespace cogip {

namespace motion_control {

/// @brief Wrapper controller that executes a wrapped controller at a reduced frequency.
///
/// This controller acts as a decorator/wrapper around another controller,
/// executing it only every N engine cycles (where N is the period_divider).
/// This allows different controllers in a chain to run at different frequencies.
///
/// Example: If the engine runs at 10ms and period_divider=5, the wrapped
/// controller will execute every 50ms (once every 5 engine cycles).
class ThrottledController : public BaseController
{
  public:
    /// @brief Constructor for the throttled controller.
    /// @param wrapped_controller The controller to wrap and throttle
    /// @param period_divider Execute wrapped controller every N cycles (must be >= 1)
    explicit ThrottledController(BaseController* wrapped_controller, uint16_t period_divider = 1);

    /// @brief Execute the wrapped controller if the period has elapsed.
    /// @param io Controllers input/output datas shared across controllers
    void execute(ControllersIO& io) override;

    /// @brief Get the wrapped controller.
    /// @return Pointer to the wrapped controller
    BaseController* wrapped_controller() const
    {
        return wrapped_controller_;
    };

    /// @brief Get the period divider.
    /// @return Period divider value
    uint16_t period_divider() const
    {
        return period_divider_;
    };

    /// @brief Set the period divider.
    /// @param period_divider New period divider (must be >= 1)
    void set_period_divider(uint16_t period_divider);

    /// @brief Get the current cycle count.
    /// @return Current cycle count
    uint16_t current_count() const
    {
        return current_count_;
    };

    /// @brief Reset the cycle counter.
    void reset_counter()
    {
        current_count_ = 0;
    };

  private:
    /// Wrapped controller to execute at reduced frequency
    BaseController* wrapped_controller_;

    /// Execute wrapped controller every N cycles (1 = every cycle, 2 = every other cycle, etc.)
    uint16_t period_divider_;

    /// Current cycle counter
    uint16_t current_count_;
};

} // namespace motion_control

} // namespace cogip

/// @}
