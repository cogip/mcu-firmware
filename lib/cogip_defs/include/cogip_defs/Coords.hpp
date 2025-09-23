// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_cogip_defs
/// @{
/// @file
/// @brief       Coords declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "PB_Coords.hpp"

namespace cogip {

namespace cogip_defs {

/// Absolute coordinates along X and Y axis
class Coords {
public:
    /// Constructor.
    explicit Coords(
        float x=0.0,       ///< [in] X coordinate
        float y=0.0        ///< [in] Y coordinate
        ) : x_(x), y_(y) {};

    /// Constructor from Protobuf class
    explicit Coords(const PB_Coords &coords) : x_(coords.get_x()), y_(coords.get_y()) {};

    /// Return X coordinate.
    float x(void) const { return x_; };

    /// Return Y coordinate.
    float y(void) const { return y_; };

    /// Set X coordinate.
    void set_x(
        float x            ///< [in] new X coordinate
        ) { x_ = x; };

    /// Set Y coordinate.
    void set_y(
        float y            ///< [in] new Y coordinate
        ) { y_ = y; };

    /// Compute the distance the destination point.
    float distance(
        const Coords &dest  ///< [in] destination
        ) const;

    /// Check if this point is placed on a segment defined by two points A,B.
    /// @return true if on [AB], false otherwise
    bool on_segment(
        const Coords &a,    ///< [in] point A
        const Coords &b     ///< [in] point A
        ) const;

    /// Check if this point is equal to another.
    /// @return true if points are equal, false otherwise
    bool operator == (
        const Coords other  ///< [in] point to compare
        ) const { return x_ == other.x_ && y_ == other.y_; };

    /// Copy data to Protobuf message.
    void pb_copy(
        PB_Coords &coords   ///< [out] Protobuf message to fill
        ) const {
        coords.set_x(x_);
        coords.set_y(y_);
    };

protected:
    float x_;              ///< x-position
    float y_;              ///< y-position
};

} // namespace cogip_defs

} // namespace cogip

/// @}
