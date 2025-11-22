// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

#include "parameter/ParameterInterface.hpp"

namespace cogip {

namespace localization {

using namespace cogip::parameter;

struct LocalizationDifferentialParameters
{
    /// @brief Constructs the Odometry parameter aggregation structure
    /// @param left_wheel_diameter_mm Left encoder wheel diameter parameter
    /// @param right_wheel_diameter_mm Right encoder wheel diameter parameter
    /// @param track_width_mm Distance between encoder wheels parameter
    /// @param left_polarity Encoder left wheel polarity
    /// @param right_polarity Encoder right wheel polarity
    LocalizationDifferentialParameters(const ParameterInterface<float>& left_wheel_diameter_mm,
                                       const ParameterInterface<float>& right_wheel_diameter_mm,
                                       const ParameterInterface<float>& track_width_mm,
                                       const ParameterInterface<float>& left_polarity,
                                       const ParameterInterface<float>& right_polarity)
        : left_wheel_diameter_mm(left_wheel_diameter_mm),
          right_wheel_diameter_mm(right_wheel_diameter_mm), track_width_mm(track_width_mm),
          left_polarity(left_polarity), right_polarity(right_polarity)
    {
    }

    /// Read only-parameter. Each parameter have it's own getter.
    const ParameterInterface<float>& left_wheel_diameter_mm;
    const ParameterInterface<float>& right_wheel_diameter_mm;
    const ParameterInterface<float>& track_width_mm;
    const ParameterInterface<float>& left_polarity;
    const ParameterInterface<float>& right_polarity;
};

} // namespace localization

} // namespace cogip
