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
#include "parameter/ParameterInterface.hpp"

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
    /// @brief OTOS configuration parameters.
    /// @details Holds references to the live linear / angular calibration
    /// scalars; any runtime change to these parameters is picked up by
    /// update() through the ParameterInterface::has_changed() poll. Offsets
    /// describe the sensor-to-robot mounting geometry and stay as plain
    /// floats (measured once at assembly).
    struct Parameters
    {
        Parameters(const cogip::parameter::ParameterInterface<float>& linear_scalar_,
                   const cogip::parameter::ParameterInterface<float>& angular_scalar_,
                   float offset_x_mm_ = 0.0f, float offset_y_mm_ = 0.0f, float offset_h_deg_ = 0.0f)
            : linear_scalar(linear_scalar_), angular_scalar(angular_scalar_),
              offset_x_mm(offset_x_mm_), offset_y_mm(offset_y_mm_), offset_h_deg(offset_h_deg_)
        {
        }

        const cogip::parameter::ParameterInterface<float>& linear_scalar;
        const cogip::parameter::ParameterInterface<float>& angular_scalar;
        float offset_x_mm;
        float offset_y_mm;
        float offset_h_deg;
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

    /// @brief Read fresh data from the OTOS sensor. Also polls the
    /// calibration scalars and re-programs the chip if they were changed
    /// by the parameter handler since the last cycle.
    /// @return 0 on success, negative on error
    int update() override;

  private:
    /// @brief Push the current calibration scalars to the sensor chip if
    /// either parameter was changed since the last poll.
    void poll_scalar_changes();

    cogip::otos::OTOS& otos_;
    const Parameters params_;
    cogip::cogip_defs::Pose pose_;
    cogip::cogip_defs::Pose prev_pose_;
    cogip::cogip_defs::Polar polar_;
    bool first_update_;
};

} // namespace localization
} // namespace cogip

/// @}
