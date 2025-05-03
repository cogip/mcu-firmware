// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#pragma once

namespace cogip {
namespace localization {

/// @brief Configuration for a single‐encoder odometer.
/// @details
///   The encoder produces raw pulses; this parameter converts them into millimeters.
///   distance_mm = pulses / pulses_per_mm
struct OdometerEncoderParameters {
    float pulses_per_mm;    ///< Number of encoder pulses per millimeter of travel

    bool reverse_polarity = false;  ///< Reverse Encoder value if true
};

} // namespace localization
} // namespace cogip
