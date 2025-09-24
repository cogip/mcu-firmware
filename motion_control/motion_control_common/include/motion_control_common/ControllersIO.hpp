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

#include "log.h"
#include <cerrno>
#include <etl/hash.h>
#include <etl/optional.h>
#include <etl/set.h>
#include <etl/string.h>
#include <etl/string_view.h>
#include <etl/unordered_map.h>
#include <etl/variant.h>
#include <etl/vector.h>

namespace cogip {

namespace motion_control {

/// Status of a position to reach
/// - moving:               in motion to its destination
/// - reached:              the position has been reached
/// - intermediate_reached: a transient position has been reached
/// - blocked:              motion is blocked by an abnormal event
/// - timeout:              motion time has reached the timeout
typedef enum { moving = 0, reached, intermediate_reached, blocked, timeout } target_pose_status_t;

/// @brief Maximum number of parameters that can be stored.
/// @note Used as ETL template parameter for the underlying unordered_map.
constexpr size_t MAX_PARAMS = 32;

/// @brief Variant type that can hold any supported parameter value.
using ParamValue = etl::variant<float, double, int, bool, etl::string<32>, target_pose_status_t>;
/// @brief Hashed key type for lookup.
using ParamKey = size_t;
/// @brief String key type for parameter names.
using KeyType = etl::string_view;
/// @brief Vector of ParamKeys with maximum size.
using ParamKeyVector = etl::vector<ParamKey, MAX_PARAMS>;
/// @brief Optional of ParamValue for return of get().
using OptionalValue = etl::optional<ParamValue>;
/// @brief Maximum number of keys that can be marked read-only.
constexpr size_t MAX_READONLY = 16;

/// @class ControllersIO
/// @brief Central storage for parameters shared across chained controllers.
/// @details
///     - Uses FNV-1a 32-bit hash of string keys (`etl::fnv_1a_32`) for lookup.
///     - Allows marking keys read-only (further sets return EACCES).
///     - Tracks “dirty” keys on each call to `set()`. You can clear or take
///     those dirty keys
///       to see exactly which parameters were modified since the last snapshot.
class ControllersIO
{
  public:
    /// @brief Compute FNV-1a 32-bit hash of a string key.
    /// @param key The parameter name.
    /// @return The 32-bit hash as ParamKey.
    static ParamKey hash_key(KeyType key);

    /// @brief Set or update a parameter value.
    /// @param key   The parameter name.
    /// @param value The value to store.
    /// @return 0 on success; EACCES if the key is read-only.
    /// @note If the key is marked read-only, the write is ignored and EACCES is
    /// returned.
    int set(KeyType key, const ParamValue& value);

    /// @brief Mark a parameter key as read-only.
    /// @param key The parameter name to protect.
    void mark_readonly(KeyType key);

    /// @brief Reset all read-only marks
    void reset_readonly_markers();

    /// @brief Retrieve the raw variant value for a key.
    /// @param key The parameter name.
    /// @return An optional containing the `ParamValue` if found, or empty
    /// otherwise.
    OptionalValue get(KeyType key) const;

    /// @brief Retrieve a typed value for a key.
    /// @tparam T The expected type (must match one of the types in `ParamValue`).
    /// @param key The parameter name.
    /// @return An optional containing the value cast to `T` if present and
    /// type-matched; empty otherwise.
    template <typename T> etl::optional<T> get_as(KeyType key) const
    {
        if (auto opt = get(key)) {
            if (etl::holds_alternative<T>(*opt)) {
                return etl::get<T>(*opt);
            } else {
                LOG_ERROR("Error getting %.*s", static_cast<int>(key.size()), key.data());
            }
        }
        return {};
    }

    /// @brief Once you have run some controllers, you can call
    /// `snapshot_modified()` to get
    ///        all keys that were written since the last snapshot. Then call
    ///        `clear_modified()` to reset the “dirty” set.
    ///@{
    /// @brief Returns a vector of all ParamKeys that have been written since the
    /// last `clear_modified()`.
    ParamKeyVector snapshot_modified() const;

    /// @brief Find which keys in `new_keys` also appear in `already_written`.
    /// @param new_keys         Vector of keys newly written by one controller
    /// subtree.
    /// @param already_written  Set of keys written so far by earlier siblings.
    /// @return A vector of ParamKey that appear in both.
    static ParamKeyVector find_collisions(const ParamKeyVector& new_keys,
                                          const etl::set<ParamKey, MAX_PARAMS>& already_written);

    /// @brief Compute the set‐difference “after \ before” for two lists of keys.
    /// @param before_ctrl All keys present before running a subtree.
    /// @param after_ctrl  All keys present after running that subtree.
    /// @return A vector containing keys in `after_ctrl` but not in `before_ctrl`.
    static ParamKeyVector difference(const ParamKeyVector& after_ctrl,
                                     const ParamKeyVector& before_ctrl);

    /// @brief Clears the internal “dirty” set, so subsequent sets will be newly
    /// recorded.
    void clear_modified();
    ///@}

  private:
    etl::unordered_map<ParamKey, ParamValue, MAX_PARAMS> data_; ///< Underlying storage map
    etl::set<ParamKey, MAX_READONLY> readonly_keys_;            ///< Set of protected keys

    /// @brief Tracks which keys have been written by calls to `set(...)`
    etl::set<ParamKey, MAX_PARAMS> _modified_keys;
};

} // namespace motion_control

} // namespace cogip

/// @}
