// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_cogip_defs
/// @{
/// @file
/// @brief       Polar class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "PB_Polar.hpp"

namespace cogip {

namespace cogip_defs {

/// Polar coordinate
class Polar
{
  public:
    /// Constructor.
    explicit Polar(float distance = 0.0, ///< [in] distance
                   float angle = 0.0     ///< [in] angle
                   )
        : distance_(distance), angle_(angle){};

    /// Constructor from Protobuf class
    explicit Polar(const PB_Polar& polar)
        : distance_(polar.get_distance()), angle_(polar.get_angle()){};

    /// Return distance.
    float distance(void) const
    {
        return distance_;
    };

    /// Return angle.
    float angle(void) const
    {
        return angle_;
    };

    /// Set distance.
    void set_distance(float distance ///< [in] new distance
    )
    {
        distance_ = distance;
    };

    /// Set angle.
    void set_angle(float angle ///< [in] new angle
    )
    {
        angle_ = angle;
    };

    /// Copy data to Protobuf message.
    void pb_copy(PB_Polar& polar ///< [out] Protobuf message to fill
    ) const
    {
        polar.set_distance(distance_);
        polar.set_angle(angle_);
    };

    /// Reverse distance
    void reverse_distance()
    {
        distance_ *= -1;
    }

    /// Reverse angle
    void reverse_angle()
    {
        if (angle_ < 0) {
            angle_ += 180;
        } else {
            angle_ -= 180;
        }
    };

    /// Reverse distance and angle
    void reverse()
    {
        reverse_distance();
        reverse_angle();
    };

  private:
    float distance_; ///< distance
    float angle_;    ///< angle
};

} // namespace cogip_defs

} // namespace cogip

/// @}
