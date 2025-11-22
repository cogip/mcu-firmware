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

/// @brief Read/Write access policy - allows both read and write operations
/// @details This is the default policy that permits all parameter operations.
struct ReadWrite
{
    /// @brief Check if write operations are allowed
    /// @return true Always returns true (write is allowed)
    static constexpr bool canWrite()
    {
        return true;
    }
};

/// @brief Read-only access policy - allows only read operations
/// @details This policy prevents any write operations on the parameter.
///          Attempts to write (via set() or pb_read()) will fail.
struct ReadOnly
{
    /// @brief Check if write operations are allowed
    /// @return false Always returns false (write is not allowed)
    static constexpr bool canWrite()
    {
        return false;
    }
};

} // namespace parameter
} // namespace cogip

/// @}
