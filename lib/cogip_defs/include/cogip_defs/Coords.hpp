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

namespace cogip {

namespace cogip_defs {

/// Absolute coordinates along X and Y axis
class Coords {
public:
    /// Constructor.
    Coords(
        double x=0.0,       ///< [in] X coordinate
        double y=0.0        ///< [in] Y coordinate
        ) : x_(x), y_(y) {};

    /// Return X coordinate.
    double x(void) const { return x_; };

    /// Return Y coordinate.
    double y(void) const { return y_; };

    /// Set X coordinate.
    void set_x(
        double x            ///< [in] new X coordinate
        ) { x_ = x; };

    /// Set Y coordinate.
    void set_y(
        double y            ///< [in] new Y coordinate
        ) { y_ = y; };

    /// Compute the distance the destination point.
    double distance(
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

protected:
    double x_;              ///< x-position
    double y_;              ///< y-position
};

} // namespace cogip_defs

} // namespace cogip

/// @}
