// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @file
/// @brief      Storage policies for parameter flash persistence
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

#include "flash_kv_storage/FlashKVStorage.hpp"

namespace cogip {
namespace parameter {

/// @brief Flash storage policy — persists parameter value to FlashDB KVDB
/// @tparam KeyHash Compile-time 32-bit FNV-1a hash identifying this parameter
///
/// @details This policy provides three hooks:
///   - `on_init(T& value)`: loads the stored value from flash (if present)
///   - `on_commit(const T& value)`: saves the current value to flash after successful validation
///   - `on_clear()`: erases the stored value from flash
///
/// Flash write failure does NOT reject an in-memory parameter update (on_commit is void).
template <uint32_t KeyHash> struct WithFlashStorage
{
    /// @brief Load parameter value from flash storage
    /// @tparam T The parameter value type
    /// @param value Reference to value (overwritten if flash contains a valid entry)
    /// @return true if a value was successfully loaded from flash
    template <typename T> static bool on_init(T& value)
    {
        return flash_kv_storage::FlashKVStorage::instance().load(KeyHash, value) == 0;
    }

    /// @brief Save parameter value to flash storage
    /// @tparam T The parameter value type
    /// @param value The validated value to persist
    template <typename T> static void on_commit(const T& value)
    {
        flash_kv_storage::FlashKVStorage::instance().store(KeyHash, value);
    }

    /// @brief Erase parameter value from flash storage
    static void on_clear()
    {
        flash_kv_storage::FlashKVStorage::instance().del(KeyHash);
    }
};

} // namespace parameter
} // namespace cogip

/// @}
