// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Policy-based parameter class template with composable policies
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

#include "etl/type_list.h"
#include "etl/type_traits.h"
#include "mutex.h"

#include "AccessPolicies.hpp"
#include "PB_ParameterCommands.hpp"
#include "ParameterInterface.hpp"
#include "ParameterRange.hpp"
#include "PolicyTraits.hpp"

namespace cogip {

namespace parameter {

/// @brief Value types a parameter can hold, in the order of the PB_ParameterValue oneof fields
using ParameterValueTypes =
    etl::type_list<float, double, int32_t, uint32_t, int64_t, uint64_t, bool>;

/// @brief Policy-based parameter class template with variadic composable policies
/// @tparam T The C++ value type, one of ParameterValueTypes
/// @tparam Policies... Zero or more policies implementing on_set(const T&) -> bool
///
/// @details Accepted range is given at construction, every value of T by default.
///          Values outside are rejected.
///          Policies implement a single optional hook:
///   - `static bool on_set(const T& value)` - Called on set, validates value without modifying it
///     Return false to reject, true to accept. Policies are chained with AND semantics.
///
/// @example
/// @code
/// // Any value of T (NaN and infinities rejected)
/// Parameter<float> speed{100.0f};
///
/// // Rejects values outside [0, 500]
/// Parameter<float> max_speed{250.0f, {0.0f, 500.0f}};
///
/// // Rejects negative values
/// Parameter<float> kp{1.0f, non_negative<float>};
///
/// // Read-only parameter
/// Parameter<float, ReadOnly> firmware_version{1.0f};
/// @endcode
template <typename T, typename... Policies> class Parameter : public ParameterInterface<T>
{
    /// Compile-time type checking: only the types PB_ParameterValue can carry are supported
    static_assert(etl::type_list_contains_v<ParameterValueTypes, T>,
                  "Parameter<T> only supports float, double, (u)int32_t, (u)int64_t and bool");

  public:
    /// @brief Constructor with initial value and range
    /// @param initial_value The initial value (checked against the range and all policies)
    /// @param range Accepted range, every value of T by default
    explicit Parameter(const T& initial_value, const ParameterRange<T>& range = ParameterRange<T>{})
        : value_(initial_value), default_value_(initial_value), range_(range), valid_(false),
          changed_(true), mutex_(MUTEX_INIT)
    {
        mutex_init(&mutex_);
        valid_ = accepts(value_);
    }

    /// @brief Set the parameter value with combined policy enforcement
    /// @param value The new value to set
    /// @return true if value was set successfully (all policies returned true)
    bool set(const T& value) override
    {
        mutex_lock(&mutex_);
        valid_ = accepts(value);
        if (valid_) {
            value_ = value;
            changed_ = true;
        }
        const bool result = valid_;
        mutex_unlock(&mutex_);

        if (result) {
            combined_on_commit<T, Policies...>(value);
        }
        return result;
    }

    /// @brief Check whether the value has changed since the last clear_changed().
    bool has_changed() const override
    {
        mutex_lock(&mutex_);
        bool result = changed_;
        mutex_unlock(&mutex_);
        return result;
    }

    /// @brief Reset the changed flag after a consumer has handled the new value.
    void clear_changed() const override
    {
        mutex_lock(&mutex_);
        changed_ = false;
        mutex_unlock(&mutex_);
    }

    /// @brief Load value from persistent storage and re-validate
    /// @return true if load succeeded (or no storage policy exists)
    /// @note If flash contains a value that fails validation, the default is kept
    ///       and load returns false. If no storage policy or no stored value,
    ///       the default is kept and load returns true.
    bool load() override
    {
        mutex_lock(&mutex_);
        T temp = value_;
        bool result = true;
        if (combined_on_init<T, Policies...>(temp)) {
            // A value was read from flash — validate it
            if (accepts(temp)) {
                value_ = temp;
                valid_ = true;
            } else {
                // Flash value rejected by validation, keep default
                result = false;
            }
        }
        // No storage policy or no stored value → keep default (success)
        mutex_unlock(&mutex_);
        return result;
    }

    /// @brief Erase persisted value and restore the compile-time default in memory
    /// @note Fires the storage policy's on_clear hook (erases the flash entry),
    ///       restores the value captured at construction, re-applies validation /
    ///       commit policies and marks the parameter as changed.
    void reset() override
    {
        mutex_lock(&mutex_);
        combined_on_clear<Policies...>();
        value_ = default_value_;
        valid_ = accepts(value_);
        changed_ = true;
        mutex_unlock(&mutex_);
    }

    /// @brief Get the current parameter value
    /// @return The current value
    ///
    /// @note This returns the value even if the parameter is marked invalid.
    ///       Use isValid() to check validity before using the value.
    T get() const override
    {
        mutex_lock(&mutex_);
        T result = value_;
        mutex_unlock(&mutex_);
        return result;
    }

    /// @brief Check if the parameter holds a valid value
    /// @return true if the value is valid, false otherwise
    bool isValid() const override
    {
        bool result = false;

        mutex_lock(&mutex_);
        result = valid_;
        mutex_unlock(&mutex_);

        return result;
    }

    /// @brief Get the accepted range
    /// @return The range given at construction
    const ParameterRange<T>& range() const
    {
        return range_;
    }

    /// @brief Convert parameter to protobuf PB_ParameterValue message
    /// @param message The PB_ParameterValue message to convert to
    /// @return true if conversion succeeded
    ///
    /// @note Uses compile-time type dispatch to set the correct oneof field in PB_ParameterValue
    /// message.
    bool pb_copy(PB_ParameterValue& message) const override
    {
        mutex_lock(&mutex_);
        T copy = value_;
        mutex_unlock(&mutex_);

        combined_on_pb_copy<T, Policies...>(copy);

        if constexpr (etl::is_same<T, float>::value) {
            message.set_float_value(copy);
            return true;
        } else if constexpr (etl::is_same<T, double>::value) {
            message.set_double_value(copy);
            return true;
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            message.set_int32_value(copy);
            return true;
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            message.set_uint32_value(copy);
            return true;
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            message.set_int64_value(copy);
            return true;
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            message.set_uint64_value(copy);
            return true;
        } else if constexpr (etl::is_same<T, bool>::value) {
            message.set_bool_value(copy);
            return true;
        } else {
            return false;
        }
    }

    /// @brief Convert parameter from protobuf PB_ParameterValue message
    /// @param message The PB_ParameterValue message to convert from
    /// @return true if conversion succeeded, false if the message holds another value type or
    ///         a policy rejects the value
    ///
    /// @note Uses compile-time type dispatch to get the correct oneof field from PB_ParameterValue
    /// message. All policies are applied to the converted value via set().
    bool pb_read(const PB_ParameterValue& message) override
    {
        T new_value;
        bool success = false;

        if constexpr (etl::is_same<T, float>::value) {
            success = message.has_float_value();
            new_value = message.float_value();
        } else if constexpr (etl::is_same<T, double>::value) {
            success = message.has_double_value();
            new_value = message.double_value();
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            success = message.has_int32_value();
            new_value = message.int32_value();
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            success = message.has_uint32_value();
            new_value = message.uint32_value();
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            success = message.has_int64_value();
            new_value = message.int64_value();
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            success = message.has_uint64_value();
            new_value = message.uint64_value();
        } else if constexpr (etl::is_same<T, bool>::value) {
            success = message.has_bool_value();
            new_value = message.bool_value();
        }

        if (success) {
            combined_on_pb_read<T, Policies...>(new_value);
            return set(new_value); // Apply validation (set() is already mutex-protected)
        }
        return false;
    }

  private:
    /// @brief Check a value against the range, then all policies
    /// @param value The value to check
    /// @return true if the value is accepted
    bool accepts(const T& value) const
    {
        return range_.contains(value) && combined_on_set<T, Policies...>(value);
    }

    T value_;                       ///< Parameter value
    const T default_value_;         ///< Default captured at construction, used by reset()
    const ParameterRange<T> range_; ///< Accepted range
    bool valid_;                    ///< Validity flag
    mutable bool changed_;          ///< True if value was (re)set since the last clear_changed()
    mutable mutex_t mutex_;         ///< Mutex for thread-safe access
};

} // namespace parameter
} // namespace cogip

/// @}
