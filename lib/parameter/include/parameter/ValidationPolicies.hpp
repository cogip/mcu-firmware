// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ValidationPolicies.hpp
/// @brief Validation policies for parameter values
///
/// These policies provide compile-time behavior composition for parameter
/// validation. They use static methods to enable zero-overhead abstraction.

#pragma once

#include "etl/algorithm.h"

namespace cogip {

namespace parameter {

/// @brief No validation policy - accepts all values as-is
/// @details This is the default policy that performs no validation.
///          The value is always considered valid.
struct NoValidation
{
    /// @brief Validate and potentially transform a parameter value
    /// @tparam T The parameter value type
    /// @param value The input value to validate
    /// @param out The output value (same as input for NoValidation)
    /// @return true Always returns true (value is always valid)
    template <typename T> static bool validate(const T& value, T& out)
    {
        out = value;
        return true;
    }
};

/// @brief Bounds validation policy - rejects values outside [min, max] range
/// @tparam MinVal Minimum allowed value (compile-time constant)
/// @tparam MaxVal Maximum allowed value (compile-time constant)
/// @details This policy rejects values that are outside the specified bounds.
///          If the value is out of range, validation fails and the parameter remains unchanged.
///          Only applicable to numeric types.
///
/// @note Compile-time checks:
///       - MinVal must be less than or equal to MaxVal (enforced via static_assert)
///       - Initial values in constructors are only checked at runtime
template <auto MinVal, auto MaxVal> struct WithBounds
{
    // Compile-time validation of bounds
    static_assert(MinVal <= MaxVal, "MinVal must be less than or equal to MaxVal");

    /// @brief Validate a parameter value against specified bounds
    /// @tparam T The parameter value type
    /// @param value The input value to validate
    /// @param out The output value (set only if validation succeeds)
    /// @return true if value is within [MinVal, MaxVal], false otherwise
    template <typename T> static bool validate(const T& value, T& out)
    {
        if (value < static_cast<T>(MinVal) || value > static_cast<T>(MaxVal)) {
            return false;
        }
        out = value;
        return true;
    }
};

} // namespace parameter

} // namespace cogip
