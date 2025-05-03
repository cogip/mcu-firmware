// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "odometer/OdometerEncoder.hpp"

namespace cogip {
namespace localization {

OdometerEncoder::OdometerEncoder(
    const OdometerEncoderParameters& parameters,
    cogip::encoder::EncoderInterface& encoder
) : params_(parameters),
    encoder_(encoder),
    distance_mm_(0.0f),
    delta_mm_(0.0f)
{}

int OdometerEncoder::init() {
    return encoder_.init();
}

void OdometerEncoder::set_distance_mm(float distance_mm) {
    distance_mm_ = distance_mm;
    delta_mm_    = 0.0f;
}

float OdometerEncoder::distance_mm() const {
    return distance_mm_;
}

float OdometerEncoder::delta_distance_mm() const {
    return delta_mm_;
}

int OdometerEncoder::update() {
    // 1) Read pulses and reset encoder counter
    int32_t pulses = encoder_.read_and_reset() * (params_.reverse_polarity ? -1 : 1);

    // 2) Convert to millimeters
    delta_mm_ = pulses / params_.pulses_per_mm;

    // 3) Accumulate
    distance_mm_ += delta_mm_;

    return 0;  // success
}

} // namespace localization
} // namespace cogip
