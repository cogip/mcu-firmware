// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

#include "OdometerEncoderParameters.hpp"
#include "OdometerInterface.hpp"
#include "encoder/EncoderInterface.hpp"

namespace cogip {
namespace localization {

/// @brief 1D odometer driven by a single encoder.
/// @details
///   Implements @ref OdometerInterface: accumulates distance in millimeters
///   and reports the incremental delta on each update().
class OdometerEncoder : public OdometerInterface
{
  public:
    /// @brief Construct a new OdometerEncoder.
    /// @param parameters   Static configuration (conversion factor).
    /// @param encoder      Hardware encoder (must implement read-and-reset).
    explicit OdometerEncoder(const OdometerEncoderParameters& parameters,
                             cogip::encoder::EncoderInterface& encoder);

    /// @copydoc OdometerInterface::init()
    int init() override;

    /// @copydoc OdometerInterface::set_distance_mm(float)
    void set_distance_mm(float distance_mm) override;

    /// @copydoc OdometerInterface::distance_mm() const
    float distance_mm() const override;

    /// @copydoc OdometerInterface::delta_distance_mm() const
    float delta_distance_mm() const override;

    /// @copydoc OdometerInterface::update()
    int update() override;

  private:
    /// @brief Conversion factor storage
    const OdometerEncoderParameters& params_;

    /// @brief Underlying encoder device
    cogip::encoder::EncoderInterface& encoder_;

    /// @brief Total accumulated distance (mm)
    float distance_mm_ = 0.0f;

    /// @brief Distance increment from last update (mm)
    float delta_mm_ = 0.0f;
};

} // namespace localization
} // namespace cogip
