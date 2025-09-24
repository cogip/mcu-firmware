// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

#include <cstdint>

#include "LocalizationDifferentialParameters.hpp"
#include "LocalizationInterface.hpp"
#include "encoder/EncoderInterface.hpp"

namespace cogip {

namespace localization {

class LocalizationDifferential : public LocalizationInterface
{

  public:
    /// @brief Construct a new differential localization object
    /// @param parameters Localization parameters
    explicit LocalizationDifferential(LocalizationDifferentialParameters& parameters,
                                      cogip::encoder::EncoderInterface& left_encoder,
                                      cogip::encoder::EncoderInterface& right_encoder)
        : parameters_(parameters), left_encoder_(left_encoder), right_encoder_(right_encoder)
    {
    }

    /// @brief Set the default localization pose
    ///
    /// @note this function should be called to reset robot pose and defined a new
    /// default one
    ///
    /// @param x X coordinate (mm)
    /// @param y Y coordinate (mm)
    /// @param O angle (deg)
    void set_pose(float x, float y, float O) override;

    /// @brief Set the default localization pose
    ///
    /// @note this function should be called to reset robot pose and defined a new
    /// default one
    ///
    /// @param pose position reference
    void set_pose(const cogip::cogip_defs::Pose& pose) override;

    /// @brief Get current pose using cogip def format
    /// @note Data units:
    ///         - x, y: mm
    ///         - O: deg
    /// @return pose cogip::cogip_defs::Pose current pose reference
    const cogip::cogip_defs::Pose& pose() override
    {
        return pose_;
    }

    /// @brief Get current polar pose delta cogip def format
    /// @note Data units:
    ///         - linear: mm
    ///         - O: deg
    /// @return velocity cogip::cogip_defs::Polar current polar pose delta
    /// reference
    const cogip::cogip_defs::Polar& delta_polar_pose() override
    {
        return polar_;
    }

    /// @brief update new robot pose (x, y, O)
    /// @return int 0 on success, negative on failure.
    int update() override;

  private:
    LocalizationDifferentialParameters& parameters_;

    cogip::encoder::EncoderInterface& left_encoder_;
    cogip::encoder::EncoderInterface& right_encoder_;

    cogip::cogip_defs::Pose pose_;
    cogip::cogip_defs::Polar polar_;
};

} // namespace localization

} // namespace cogip
