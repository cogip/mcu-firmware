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
    /// @brief OTOS configuration parameters
    struct Parameters
    {
        float linear_scalar = 1.0f;  ///< Calibration scalar (0.872 to 1.127)
        float angular_scalar = 1.0f; ///< Calibration scalar (0.872 to 1.127)
        float offset_x_mm = 0.0f;    ///< Mounting offset X (mm)
        float offset_y_mm = 0.0f;    ///< Mounting offset Y (mm)
        float offset_h_deg = 0.0f;   ///< Mounting offset heading (deg)
    };

    /// @brief Constructor
    /// @param otos Reference to the OTOS driver instance
    /// @param params OTOS configuration parameters
    explicit LocalizationOTOS(cogip::otos::OTOS& otos, const Parameters& params);

    /// @brief Set the current pose (resets OTOS tracking)
    void set_pose(float x, float y, float O) override;

    /// @brief Set the current pose (resets OTOS tracking)
    void set_pose(const cogip::cogip_defs::Pose& pose) override;

    /// @brief Get the current pose
    const cogip::cogip_defs::Pose& pose() override;

    /// @brief Get the polar delta since last update
    const cogip::cogip_defs::Polar& delta_polar_pose() override;

    /// @brief Initialize the OTOS sensor and run IMU calibration
    /// @return 0 on success, negative on error
    int init() override;

    /// @brief No-op for OTOS (tracks absolute position)
    void reset() override;

    /// @brief Read fresh data from the OTOS sensor
    /// @return 0 on success, negative on error
    int update() override;

  private:
    cogip::otos::OTOS& otos_;
    Parameters params_;
    cogip::cogip_defs::Pose pose_;
    cogip::cogip_defs::Pose prev_pose_;
    cogip::cogip_defs::Polar polar_;
    bool first_update_;
};

} // namespace localization
} // namespace cogip

/// @}
