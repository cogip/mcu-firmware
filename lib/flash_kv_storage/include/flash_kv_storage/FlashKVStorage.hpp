// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_flash_kv_storage
/// @{
/// @file
/// @brief      Singleton FlashDB KVDB wrapper for persistent key-value storage
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstddef>
#include <cstdint>
#include <etl/type_traits.h>

#include "flashdb.h"
#include "mutex.h"

namespace cogip {
namespace flash_kv_storage {

/// @brief Singleton wrapper around FlashDB KVDB for persistent parameter storage
///
/// @details Provides blob-level get/set/del operations keyed by 8-char hex strings
///          derived from 32-bit parameter key hashes.
///
/// @note Initialization is explicit: call `init()` from `main()` after hardware is ready.
///       Before `init()`, all operations silently return failure — this is safe for static
///       `Parameter` objects that attempt `on_init` during construction before `main()`.
class FlashKVStorage
{
  public:
    /// @brief Get the singleton instance
    /// @return Reference to the singleton FlashKVStorage
    static FlashKVStorage& instance();

    /// @brief Initialize the FlashDB KVDB backend
    /// @return 0 on success, negative error code on failure
    /// @note Must be called from `main()` after hardware init. Calling multiple times is safe.
    int init();

    /// @brief Check if the store has been initialized
    /// @return true if init() has been called successfully
    bool is_initialized() const
    {
        return initialized_;
    }

    /// @brief Store a trivially copyable value under the given key hash
    /// @tparam T Value type (must be trivially copyable)
    /// @param key_hash 32-bit parameter key hash
    /// @param value Reference to the value to store
    /// @return 0 on success, negative error code on failure
    template <typename T> int store(uint32_t key_hash, const T& value)
    {
        static_assert(etl::is_trivially_copyable_v<T>, "T must be trivially copyable");
        return store_blob(key_hash, &value, sizeof(T));
    }

    /// @brief Load a trivially copyable value for the given key hash
    /// @tparam T Value type (must be trivially copyable)
    /// @param key_hash 32-bit parameter key hash
    /// @param value Reference to the value to load into
    /// @return 0 on success, negative error code on failure or key not found
    template <typename T> int load(uint32_t key_hash, T& value)
    {
        static_assert(etl::is_trivially_copyable_v<T>, "T must be trivially copyable");
        return load_blob(key_hash, &value, sizeof(T));
    }

    /// @brief Delete a key from the store
    /// @param key_hash 32-bit parameter key hash
    /// @return 0 on success, negative error code on failure
    int del(uint32_t key_hash);

    // Non-copyable, non-movable
    FlashKVStorage(const FlashKVStorage&) = delete;
    FlashKVStorage& operator=(const FlashKVStorage&) = delete;

  private:
    FlashKVStorage();

    /// @brief Convert a 32-bit hash to an 8-char hex string key
    /// @param key_hash The hash value
    /// @param buf Output buffer (must be at least 9 bytes)
    static void hash_to_key(uint32_t key_hash, char* buf);

    /// @brief Store a binary blob under the given key hash
    int store_blob(uint32_t key_hash, const void* data, size_t size);

    /// @brief Load a binary blob for the given key hash
    int load_blob(uint32_t key_hash, void* data, size_t size);

    /// @brief FlashDB lock callback
    static void fdb_lock(fdb_db_t db);

    /// @brief FlashDB unlock callback
    static void fdb_unlock(fdb_db_t db);

    static constexpr const char* kDBName = "flash_kv_storage"; ///< KVDB name for FlashDB init

    struct fdb_kvdb kvdb_; ///< FlashDB KVDB instance
    mutex_t kvdb_mutex_;   ///< Mutex for KVDB thread safety
    bool initialized_;     ///< Whether KVDB has been initialized
};

} // namespace flash_kv_storage
} // namespace cogip

/// @}
