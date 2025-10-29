// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file StoragePolicies.hpp
/// @brief Storage policies for parameter persistence
///
/// These policies provide compile-time behavior composition for parameter
/// storage to non-volatile memory (e.g., flash). They use static methods
/// to enable zero-overhead abstraction.
///
/// @note This is currently a stub for future implementation of flash storage.

#pragma once

namespace cogip {

namespace parameter {

/// @brief No storage policy - parameters are not persisted
/// @details This is the default policy that provides no persistence.
///          Parameters are lost on power cycle.
struct NoStorage
{
    /// @brief Load a parameter value from storage (no-op for NoStorage)
    /// @tparam T The parameter value type
    /// @param value The parameter value to load into (unchanged)
    /// @return false Always returns false (no storage available)
    template <typename T> static bool load(T& /* value */)
    {
        return false;
    }

    /// @brief Store a parameter value to persistent storage (no-op for NoStorage)
    /// @tparam T The parameter value type
    /// @param value The parameter value to store (unused)
    /// @return false Always returns false (no storage available)
    template <typename T> static bool store(const T& /* value */)
    {
        return false;
    }
};

/// @brief Flash storage policy - persist parameters to flash memory
/// @details This policy will store parameters to flash for persistence
///          across power cycles.
/// @note FUTURE IMPLEMENTATION - Currently behaves like NoStorage
/// @todo Implement flash storage using RIOT's MTD (Memory Technology Device) API
struct WithFlashStorage
{
    /// @brief Load a parameter value from flash storage
    /// @tparam T The parameter value type
    /// @param value The parameter value to load into
    /// @return true if loaded successfully, false otherwise
    /// @note FUTURE IMPLEMENTATION
    template <typename T> static bool load(T& /* value */)
    {
        // TODO: Implement flash read using MTD
        return false;
    }

    /// @brief Store a parameter value to flash storage
    /// @tparam T The parameter value type
    /// @param value The parameter value to store
    /// @return true if stored successfully, false otherwise
    /// @note FUTURE IMPLEMENTATION
    template <typename T> static bool store(const T& /* value */)
    {
        // TODO: Implement flash write using MTD
        return false;
    }
};

} // namespace parameter

} // namespace cogip
