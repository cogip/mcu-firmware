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

#include "etl/type_traits.h"
#include "mutex.h"

#include "AccessPolicies.hpp"
#include "PB_ParameterCommands.hpp"
#include "ParameterInterface.hpp"
#include "PolicyTraits.hpp"
#include "ValidationPolicies.hpp"

namespace cogip {

namespace parameter {

/// @brief Policy-based parameter class template with variadic composable policies
/// @tparam T The C++ value type (arithmetic types or bool)
/// @tparam Policies... Zero or more policies implementing on_set(T&) -> bool
///
/// @details Policies implement a single optional hook:
///   - `static bool on_set(T& value)` - Called on set, can validate or modify value
///     Return false to reject, true to accept. Policies are chained with AND semantics.
///
/// @example
/// @code
/// // Simple parameter (read/write, no validation)
/// Parameter<float> speed{100.0f};
///
/// // Read-only parameter
/// Parameter<float, ReadOnly> firmware_version{1.0f};
///
/// // Bounded parameter (rejects out-of-range values)
/// Parameter<float, WithBounds<0, 500>> max_speed{250.0f};
///
/// // Clamping parameter (modifies out-of-range values)
/// Parameter<float, Clamp<0, 100>> percentage{150.0f}; // becomes 100.0f
///
/// // Composed: read-only AND bounded
/// Parameter<float, ReadOnly, WithBounds<0, 500>> calibration{123.0f};
/// @endcode
template <typename T, typename... Policies> class Parameter : public ParameterInterface<T>
{
    /// Compile-time type checking: only arithmetic types (integers, floats) and bool are supported
    static_assert(etl::is_arithmetic<T>::value || etl::is_same<T, bool>::value,
                  "Parameter<T> only supports arithmetic types (int, float, double) and bool");

  public:
    /// @brief Default constructor - initializes with invalid state
    Parameter() : value_{}, valid_(false), changed_(true), mutex_(MUTEX_INIT)
    {
        mutex_init(&mutex_);
    }

    /// @brief Constructor with initial value
    /// @param initial_value The initial value (will be processed through all policies)
    explicit Parameter(const T& initial_value)
        : value_(initial_value), valid_(false), changed_(true), mutex_(MUTEX_INIT)
    {
        mutex_init(&mutex_);
        valid_ = combined_on_set<T, Policies...>(value_);
    }

    /// @brief Set the parameter value with combined policy enforcement
    /// @param value The new value to set
    /// @return true if value was set successfully (all policies returned true)
    bool set(const T& value) override
    {
        mutex_lock(&mutex_);
        T temp = value;
        valid_ = combined_on_set<T, Policies...>(temp);
        if (valid_) {
            value_ = temp;
            changed_ = true;
            combined_on_commit<T, Policies...>(value_);
        }
        mutex_unlock(&mutex_);
        return valid_;
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
            if (combined_on_set<T, Policies...>(temp)) {
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
    /// @return true if conversion succeeded
    ///
    /// @note Uses compile-time type dispatch to get the correct oneof field from PB_ParameterValue
    /// message. All policies are applied to the converted value via set().
    bool pb_read(const PB_ParameterValue& message) override
    {
        T new_value;
        bool success = false;

        if constexpr (etl::is_same<T, float>::value) {
            new_value = message.float_value();
            success = true;
        } else if constexpr (etl::is_same<T, double>::value) {
            new_value = message.double_value();
            success = true;
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            new_value = message.int32_value();
            success = true;
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            new_value = message.uint32_value();
            success = true;
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            new_value = message.int64_value();
            success = true;
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            new_value = message.uint64_value();
            success = true;
        } else if constexpr (etl::is_same<T, bool>::value) {
            new_value = message.bool_value();
            success = true;
        }

        if (success) {
            combined_on_pb_read<T, Policies...>(new_value);
            return set(new_value); // Apply validation (set() is already mutex-protected)
        }
        return false;
    }

  private:
    T value_;               ///< Parameter value
    bool valid_;            ///< Validity flag
    mutable bool changed_;  ///< True if value was (re)set since the last clear_changed()
    mutable mutex_t mutex_; ///< Mutex for thread-safe access
};

/// @brief Compile-time introspection of a `Parameter<T, Policies...>`.
///
/// @details Specialized on Parameter so callers (ParameterDescriptor / DECLARE_PARAM
///          macros) can auto-derive PB type, read-only flag, and optional
///          bounds without re-declaring them at each registration site.
template <typename P> struct parameter_traits;

template <typename T, typename... Policies> struct parameter_traits<Parameter<T, Policies...>>
{
    using value_type = T;
    static constexpr PB_ParameterType pb_type = pb_type_of<T>();
    static constexpr bool read_only = is_read_only_v<Policies...>;
    static constexpr bool has_bounds = has_bounds_v<Policies...>;
    static constexpr auto min_value = bounds_min_v<Policies...>;
    static constexpr auto max_value = bounds_max_v<Policies...>;
};

} // namespace parameter
} // namespace cogip

/// @}
