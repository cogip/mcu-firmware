// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

#include <cstdint>

namespace cogip {
namespace localization {

/// @brief 1D odometer interface: tracks a single linear distance (e.g.,
/// elevator travel).
/// @details
///   Unlike LocalizationInterface, this interface only manages a single
///   distance value (millimeters) and its increment between two update() calls.
class OdometerInterface
{
  public:
    virtual ~OdometerInterface() = default;

    /// @brief Initialize odometer.
    /// @return 0 on success, negative value on failure.
    virtual int init() = 0;

    /// @brief Set the current distance (in millimeters).
    /// @note Use this to reset or re-home the odometer.
    /// @param distance_mm New absolute distance in millimeters.
    virtual void set_distance_mm(float distance_mm) = 0;

    /// @brief Get the current absolute distance (in millimeters).
    /// @return Current accumulated distance in millimeters.
    virtual float distance_mm() const = 0;

    /// @brief Get the distance change since the last update (in millimeters).
    /// @return Delta distance (can be negative) in millimeters.
    virtual float delta_distance_mm() const = 0;

    /// @brief Update internal counters by reading sensors.
    /// @return 0 on success, negative value on failure.
    virtual int update() = 0;
};

} // namespace localization
} // namespace cogip
