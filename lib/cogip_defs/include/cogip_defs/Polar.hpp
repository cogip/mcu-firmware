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

namespace cogip {

namespace cogip_defs {

/// Polar coordinate
class Polar {
public:
    /// Constructor.
    Polar(
        double distance = 0.0, ///< [in] distance
        double angle = 0.0     ///< [in] angle
        ) : distance_(distance), angle_(angle) {};

    /// Return distance.
    double distance(void) const { return distance_; };

    /// Return angle.
    double angle(void) const { return angle_; };

    /// Set distance.
    void set_distance(
        double distance        ///< [in] new distance
        ) { distance_ = distance; };

    /// Set angle.
    void set_angle(
        double angle           ///< [in] new angle
        ) { angle_ = angle; };

private:
    double distance_;          ///< distance
    double angle_;             ///< angle
};

} // namespace cogip_defs

} // namespace cogip

/// @}
