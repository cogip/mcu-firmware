// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    motion_control_common
/// @{
/// @file
/// @brief      Controllers input/output global storage
/// @author     Gilles DOFFE <g.doffe@gmail.com>

#pragma once

#include <etl/unordered_map.h>
#include <etl/variant.h>
#include <etl/set.h>
#include <etl/hash.h>
#include <etl/string.h>
#include <etl/optional.h>

/// @file ControllersIO.hpp
/// @brief Declaration of ControllersIO for managing shared I/O parameters between controllers.
/// @details Supports hashed string keys, read‑only protection, and optional‑based access.

/// @brief Variant type that can hold any supported parameter value.
/// @details Extend by adding types to the variant list if needed.
using IOValue = etl::variant<float, int, bool, etl::string<32>>;

/// @brief Hashed key type for parameter lookup.
using IOKey = size_t;

/// @brief Maximum number of parameters that can be stored.
constexpr size_t MAX_PARAMS   = 32;
/// @brief Maximum number of keys that can be marked read‑only.
constexpr size_t MAX_READONLY = 16;

/// @brief Optional of IOValue for return of get().
using OptionalIOValue = etl::optional<IOValue>;

/// @class ControllersIO
/// @brief Central storage for parameters shared across chained controllers.
/// @details
///   - Uses FNV‑1a hash of string keys for lookup.
///   - Allows marking keys read‑only (further sets are ignored).
///   - Provides `get()` and `get_as<T>()` returning optionals (no exceptions).
class ControllersIO {
public:
    /// @brief Compute FNV‑1a hash of a string key.
    /// @param key The parameter name.
    /// @return The resulting hash as IOKey.
    static IOKey hash_key(const etl::string_view key);

    /// @brief Set or update a parameter value.
    /// @param key   The parameter name.
    /// @param value The value to store.
    /// @note If the key is marked read‑only, the write is ignored.
    void set(const etl::string_view key, const IOValue& value);

    /// @brief Mark a parameter key as read‑only.
    /// @param key The parameter name to protect.
    /// @note Subsequent calls to `set()` for this key are ignored.
    void set_readonly(const etl::string_view key);

    /// @brief Retrieve the raw variant value for a key.
    /// @param key The parameter name.
    /// @return An optional containing the 'IOValue' if found, or empty otherwise.
    OptionalIOValue get(const etl::string_view key) const;

    /// @brief Retrieve a typed value for a key.
    /// @tparam T The expected type (must match one of the types in 'IOValue').
    /// @param key The parameter name.
    /// @return An optional containing the value cast to 'T' if present and type-matched; empty otherwise.
    template <typename T>
    etl::optional<T> get_as(const etl::string_view key) const;

private:
    etl::unordered_map<IOKey, IOValue, MAX_PARAMS>  data_;          ///< Underlying storage map
    etl::set<IOKey, MAX_READONLY>                   readonly_keys_; ///< Set of protected keys
};

/// @}
