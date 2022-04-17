// Copyright (C) 2021 COGIP Robotics association
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    platforms_abstract Abstract platform
/// @ingroup     platforms
/// @{
/// @file
/// @author      Gilles DOFFE <g.doffe@gmail.com>
/// @author      Eric Courtois <eric.courtois@gmail.com>

#include <cstdint>

/// Abstract platform.
class Platform {
public:
    /// Constructor.
    Platform() {};

    ///< Robot ID for logs
    virtual uint8_t robot_id() const = 0;

    /// @name Robot mechanical properties
    ///
    /// To be computed :
    ///  - PULSE_PER_MM          : Number of pulses per mm of coding wheel
    ///  - WHEELS_DISTANCE       : Distance between coding wheels (pulses)
    ///  - PULSE_PER_DEGREE      : Number of pulses per degree of coding wheel
    ///
    /// Must be known :
    ///  - WHEELS_DIAMETER       : Coding wheel diameter (mm)
    ///  - WHEELS_DISTANCE_MM    : Distance between coding wheels (mm)
    ///  - WHEELS_ENCODER_RESOLUTION: Number of pulses by turn of coding wheels
    ///
    /// Intermediate calculation:
    ///  - WHEELS_PERIMETER = pi*WHEELS_DIAMETER
    ///  - PULSE_PER_MM = WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER
    ///
    ///  - WHEELS_DIAMETER = 60 mm
    ///  - WHEELS_DISTANCE_MM = 280 mm
    ///  - WHEELS_ENCODER_RESOLUTION = 2000
    ///
    /// @{

    /// Robot width (mm).
    virtual float robot_width() const = 0;

    /// Point the most far from robot center (mm).
    virtual float robot_margin() const = 0;

    /// Size of the beacon support (a cylinder of 70mm diameter to a cube of 100mm width).
    virtual float beacon_support_diameter() const = 0;

    /// WHEELS_ENCODER_RESOLUTION / WHEELS_PERIMETER.
    virtual float pulse_per_mm() const = 0;

    /// WHEELS_DISTANCE_MM * PULSE_PER_MM.
    virtual float wheels_distance() const = 0;

    /// WHEELS_DISTANCE * 2 * PI / 360.
    virtual float pulse_per_degree() const = 0;

    /// @}

    /// @name Shell
    /// @{

    /// Delay to press a key before the robot starts.
    virtual uint8_t start_coundown() const = 0;

    /// @}
};
/// @}
