// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    target_change_detector Target Change Detector parameters
/// @{
/// @file
/// @brief      Target Change Detector parameters
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

namespace cogip {

namespace motion_control {

/// @brief Target Change Detector parameters
///
/// Carries the epsilon used when comparing floating-point watched keys via
/// `areFloatsEqual`. Integer and boolean keys are compared exactly regardless
/// of this value.
class TargetChangeDetectorParameters
{
  public:
    /// @brief Constructor
    /// @param epsilon Tolerance for float comparison (default 1e-3).
    ///                For mm/deg targets this filters serialisation noise
    ///                (e.g. -400.00000000000006 vs -400.0) while staying
    ///                well below any meaningful motion.
    explicit TargetChangeDetectorParameters(float epsilon = 1e-3f) : epsilon_(epsilon) {}

    /// @brief Tolerance used for float comparison.
    float epsilon() const
    {
        return epsilon_;
    }

  private:
    float epsilon_; ///< Epsilon used by areFloatsEqual
};

} // namespace motion_control

} // namespace cogip

/// @}
