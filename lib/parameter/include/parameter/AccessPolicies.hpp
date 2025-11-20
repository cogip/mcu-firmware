// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Access policies for parameter read/write control
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

namespace cogip {

namespace parameter {

/// @brief Read-only access policy - prevents all write operations
/// @details This policy rejects any write operations on the parameter.
///          Attempts to write (via set() or pb_read()) will fail.
struct ReadOnly
{
    /// @brief Reject any write attempt
    /// @tparam T The parameter value type
    /// @param value The value (ignored)
    /// @return false Always returns false
    template <typename T> static bool on_set([[maybe_unused]] T& value)
    {
        return false;
    }
};

} // namespace parameter
} // namespace cogip

/// @}
