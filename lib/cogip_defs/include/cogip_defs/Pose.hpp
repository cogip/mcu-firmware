// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_cogip_defs
/// @{
/// @file
/// @brief       Pose class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "cogip_defs/Coords.hpp"
#include "cogip_defs/Polar.hpp"
#include "PB_Pose.hpp"
#include "trigonometry.h"

namespace cogip {

namespace cogip_defs {

/// A robot position
class Pose : public Coords {
public:
    /// Constructor.
    Pose(
        float x=0.0,         ///< [in] X coordinate
        float y=0.0,         ///< [in] Y coordinate
        float O=0.0          ///< [in] 0-orientation
        ) : Coords(x, y), O_(O) {};

    /// Constructor from Protobuf class
    explicit Pose(const PB_Pose &pose) : Coords(pose.get_x(), pose.get_y()), O_(pose.get_O()) {};

    /// Return coordinates.
    Coords coords(void) const { return Coords(x_, y_); };

    /// Set coordinates.
    void set_coords(
        const Coords &coords  ///< [in] new coordinates
        ) { x_ = coords.x(); y_ = coords.y();};

    /// Return 0-orientation.
    float O(void) const { return O_; };

    /// Set 0-orientation.
    void set_O(
        float O              ///< [in] new 0-orientation
        ) { O_ = O; };

    /// Check if this pose is equal to another.
    /// @return true if poses are equal, false otherwise
    bool operator == (
        const Pose other      ///< [in] pose to compare
        ) const { return x_ == other.x_ && y_ == other.y_ && O_ == other.O_; };

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Pose &pose         ///< [out] Protobuf message to fill
        ) const {
        pose.set_x(x_);
        pose.set_y(y_);
        pose.set_O(O_);
    };

    Polar operator-(const Pose& p) {
        float error_x = x_ - p.x();
        float error_y = y_ - p.y();

        float error_O = limit_angle_rad(atan2(error_y, error_x) - DEG2RAD(p.O()));

        return Polar(
            sqrt(square(error_x) + square(error_y)),
            RAD2DEG(error_O)
        );
    };

protected:
    float O_;                ///< 0-orientation
};

} // namespace cogip_defs

} // namespace cogip

/// @}
