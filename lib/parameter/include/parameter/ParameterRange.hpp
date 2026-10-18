// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @file
/// @brief      Range of the values accepted by a parameter
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "etl/algorithm.h"
#include "etl/limits.h"

namespace cogip {

namespace parameter {

/// @brief Range [min, max] of the values accepted by a parameter
/// @tparam T The parameter value type
template <typename T> class ParameterRange
{
  public:
    /// @brief Range [min, max], e.g. `ParameterRange<float>{0.0f, 500.0f}`
    /// @param min Lowest accepted value
    /// @param max Highest accepted value
    constexpr ParameterRange(const T& min, const T& max) : min_(min), max_(max) {}

    /// @brief Range covering every value of T
    constexpr ParameterRange()
        : ParameterRange(etl::numeric_limits<T>::lowest(), etl::numeric_limits<T>::max())
    {
    }

    /// @brief Check whether a value is within the range
    /// @param value The value to check
    /// @return true if min() <= value <= max(), false otherwise (NaN included)
    constexpr bool contains(const T& value) const
    {
        return etl::clamp(value, min_, max_) == value;
    }

    /// @brief Get the lower bound
    /// @return The lowest accepted value
    constexpr T min() const
    {
        return min_;
    }

    /// @brief Get the upper bound
    /// @return The highest accepted value
    constexpr T max() const
    {
        return max_;
    }

  private:
    T min_; ///< Lowest accepted value
    T max_; ///< Highest accepted value
};

} // namespace parameter
} // namespace cogip

/// @}
