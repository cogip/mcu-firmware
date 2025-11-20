// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Validation policies for parameter value constraints
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "etl/algorithm.h"

namespace cogip {

namespace parameter {

/// @brief Bounds validation policy - rejects values outside [min, max] range
/// @tparam MinVal Minimum allowed value (compile-time constant)
/// @tparam MaxVal Maximum allowed value (compile-time constant)
/// @details This policy rejects values that are outside the specified bounds.
///          If the value is out of range, on_set returns false and the parameter remains unchanged.
template <auto MinVal, auto MaxVal> struct WithBounds
{
    static_assert(MinVal <= MaxVal, "MinVal must be less than or equal to MaxVal");

    /// @brief Validate value against specified bounds
    /// @tparam T The parameter value type
    /// @param value The value to validate (not modified)
    /// @return true if value is within [MinVal, MaxVal], false otherwise
    template <typename T> static bool on_set(T& value)
    {
        return value >= static_cast<T>(MinVal) && value <= static_cast<T>(MaxVal);
    }
};

/// @brief Clamping policy - clamps values to [min, max] range (always succeeds)
/// @tparam MinVal Minimum allowed value (compile-time constant)
/// @tparam MaxVal Maximum allowed value (compile-time constant)
/// @details This policy modifies out-of-range values to fit within bounds.
template <auto MinVal, auto MaxVal> struct Clamp
{
    static_assert(MinVal <= MaxVal, "MinVal must be less than or equal to MaxVal");

    /// @brief Clamp value to specified bounds
    /// @tparam T The parameter value type
    /// @param value The value to clamp (modified in place)
    /// @return true always (clamping never fails)
    template <typename T> static bool on_set(T& value)
    {
        if (value < static_cast<T>(MinVal)) {
            value = static_cast<T>(MinVal);
        } else if (value > static_cast<T>(MaxVal)) {
            value = static_cast<T>(MaxVal);
        }
        return true;
    }
};

/// @brief Non-zero validation policy - rejects zero values
/// @details Useful for parameters that must not be zero (e.g., divisors, multipliers)
struct NonZero
{
    /// @brief Validate that value is not zero
    /// @tparam T The parameter value type
    /// @param value The value to validate (not modified)
    /// @return true if value is not zero, false otherwise
    template <typename T> static bool on_set(T& value)
    {
        return value != T{};
    }
};

/// @brief Positive validation policy - rejects zero and negative values
/// @details Useful for parameters that must be strictly positive (e.g., speeds)
struct Positive
{
    /// @brief Validate that value is strictly positive (> 0)
    /// @tparam T The parameter value type
    /// @param value The value to validate (not modified)
    /// @return true if value is positive, false otherwise
    template <typename T> static bool on_set(T& value)
    {
        return value > T{};
    }
};

} // namespace parameter
} // namespace cogip

/// @}
