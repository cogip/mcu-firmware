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

#include "PB_Pose.hpp"

namespace cogip {

namespace cogip_defs {

/// A robot position
class Pose : public Coords {
public:
    /// Constructor.
    Pose(
        double x=0.0,         ///< [in] X coordinate
        double y=0.0,         ///< [in] Y coordinate
        double O=0.0          ///< [in] 0-orientation
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
    double O(void) const { return O_; };

    /// Set 0-orientation.
    void set_O(
        double O              ///< [in] new 0-orientation
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

private:
    double O_;                ///< 0-orientation
};

} // namespace cogip_defs

} // namespace cogip

/// @}
