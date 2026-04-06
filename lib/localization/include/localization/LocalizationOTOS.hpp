// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     localization
/// @{
/// @file
/// @brief       Localization implementation using SparkFun OTOS sensor
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include "localization/LocalizationInterface.hpp"
#include "otos/OTOS.hpp"

namespace cogip {
namespace localization {

/// @brief 2D localization using the SparkFun OTOS optical tracking sensor.
/// @details
///   Implements LocalizationInterface by reading the absolute pose from
///   the OTOS sensor and computing polar deltas for the control loop.
///   Drop-in replacement for LocalizationDifferential.
class LocalizationOTOS : public LocalizationInterface
{
  public:
    /// @brief Constructor
    /// @param otos Reference to the OTOS driver instance
    explicit LocalizationOTOS(cogip::otos::OTOS& otos);

    /// @brief Set the current pose (resets OTOS tracking)
    void set_pose(float x, float y, float O) override;

    /// @brief Set the current pose (resets OTOS tracking)
    void set_pose(const cogip::cogip_defs::Pose& pose) override;

    /// @brief Get the current pose
    const cogip::cogip_defs::Pose& pose() override;

    /// @brief Get the polar delta since last update
    const cogip::cogip_defs::Polar& delta_polar_pose() override;

    /// @brief Read fresh data from the OTOS sensor
    /// @return 0 on success, negative on error
    int update() override;

  private:
    cogip::otos::OTOS& otos_;
    cogip::cogip_defs::Pose pose_;
    cogip::cogip_defs::Pose prev_pose_;
    cogip::cogip_defs::Polar polar_;
    bool first_update_;
};

} // namespace localization
} // namespace cogip

/// @}
