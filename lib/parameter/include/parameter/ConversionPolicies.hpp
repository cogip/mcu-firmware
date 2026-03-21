// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @file
/// @brief      Unit conversion policies for protobuf interface
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "etl/type_traits.h"

namespace cogip {
namespace parameter {

/// @brief Speed unit conversion policy for protobuf interface
/// Converts between user units (/s) and internal units (/period) at the protobuf boundary.
/// Internal storage and C++ set()/get() remain in /period units.
/// @tparam PeriodMs Control loop period in milliseconds (compile-time constant)
template <int PeriodMs> struct SpeedConversion
{
    /// @brief Convert from user units (/s) to internal units (/period)
    /// Called when receiving a value from protobuf (user configures in /s)
    template <typename T> static void on_pb_read(T& value)
    {
        static_assert(etl::is_same<T, float>::value,
                      "SpeedConversion only supports float parameters");
        value = value * static_cast<T>(PeriodMs) / static_cast<T>(1000);
    }

    /// @brief Convert from internal units (/period) to user units (/s)
    /// Called when sending a value to protobuf (user sees /s)
    template <typename T> static void on_pb_copy(T& value)
    {
        static_assert(etl::is_same<T, float>::value,
                      "SpeedConversion only supports float parameters");
        value = value * static_cast<T>(1000) / static_cast<T>(PeriodMs);
    }
};

/// @brief Acceleration unit conversion policy for protobuf interface
/// Converts between user units (/s²) and internal units (/period²) at the protobuf boundary.
/// Internal storage and C++ set()/get() remain in /period² units.
/// @tparam PeriodMs Control loop period in milliseconds (compile-time constant)
template <int PeriodMs> struct AccelerationConversion
{
    /// @brief Convert from user units (/s²) to internal units (/period²)
    /// Called when receiving a value from protobuf (user configures in /s²)
    template <typename T> static void on_pb_read(T& value)
    {
        static_assert(etl::is_same<T, float>::value,
                      "AccelerationConversion only supports float parameters");
        value = value * (static_cast<T>(PeriodMs) * static_cast<T>(PeriodMs))
                / (static_cast<T>(1000) * static_cast<T>(1000));
    }

    /// @brief Convert from internal units (/period²) to user units (/s²)
    /// Called when sending a value to protobuf (user sees /s²)
    template <typename T> static void on_pb_copy(T& value)
    {
        static_assert(etl::is_same<T, float>::value,
                      "AccelerationConversion only supports float parameters");
        value = value * (static_cast<T>(1000) * static_cast<T>(1000))
                / (static_cast<T>(PeriodMs) * static_cast<T>(PeriodMs));
    }
};

} // namespace parameter
} // namespace cogip

/// @}
