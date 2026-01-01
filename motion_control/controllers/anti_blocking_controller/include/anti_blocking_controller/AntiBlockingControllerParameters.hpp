// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    anti_blocking_controller Anti-Blocking controller parameters
/// @{
/// @file
/// @brief      Anti-Blocking controller parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <cstdint>

namespace cogip {

namespace motion_control {

/// @brief Parameters for AntiBlockingController.
///
/// Blocking detection logic:
/// - If current_speed < speed_threshold AND
///   |speed_order - current_speed| > error_threshold
/// - Then increment blocked cycles counter
/// - If counter > cycles_threshold, robot is blocked
class AntiBlockingControllerParameters
{
  public:
    /// @brief Constructor with all parameters.
    /// @param enabled                 Enable/disable anti-blocking detection
    /// @param speed_threshold         Speed below which blocking can be detected
    /// @param error_threshold         Minimum speed error to consider as "not accelerating"
    /// @param cycles_threshold        Number of consecutive blocked cycles before declaring blocked
    explicit AntiBlockingControllerParameters(bool enabled = false, float speed_threshold = 0.0f,
                                              float error_threshold = 0.0f,
                                              uint16_t cycles_threshold = 0)
        : enabled_(enabled), speed_threshold_(speed_threshold), error_threshold_(error_threshold),
          cycles_threshold_(cycles_threshold)
    {
    }

    /// @brief Check if anti-blocking is enabled.
    /// @return true if enabled, false otherwise.
    bool enabled() const
    {
        return enabled_;
    }

    /// @brief Set anti-blocking enabled state.
    /// @param enabled New enabled state.
    void set_enabled(bool enabled)
    {
        enabled_ = enabled;
    }

    /// @brief Get speed threshold.
    /// @return Speed threshold value.
    float speed_threshold() const
    {
        return speed_threshold_;
    }

    /// @brief Set speed threshold.
    /// @param threshold New speed threshold.
    void set_speed_threshold(float threshold)
    {
        speed_threshold_ = threshold;
    }

    /// @brief Get error threshold.
    /// @return Error threshold value.
    float error_threshold() const
    {
        return error_threshold_;
    }

    /// @brief Set error threshold.
    /// @param threshold New error threshold.
    void set_error_threshold(float threshold)
    {
        error_threshold_ = threshold;
    }

    /// @brief Get cycles threshold.
    /// @return Cycles threshold value.
    uint16_t cycles_threshold() const
    {
        return cycles_threshold_;
    }

    /// @brief Set cycles threshold.
    /// @param threshold New cycles threshold.
    void set_cycles_threshold(uint16_t threshold)
    {
        cycles_threshold_ = threshold;
    }

  private:
    bool enabled_;              ///< Enable/disable anti-blocking detection
    float speed_threshold_;     ///< Speed below which blocking can be detected
    float error_threshold_;     ///< Minimum speed error to consider as blocked
    uint16_t cycles_threshold_; ///< Consecutive cycles before declaring blocked
};

} // namespace motion_control

} // namespace cogip

/// @}
