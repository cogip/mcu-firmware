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
    Parameter() : value_{}, valid_(false), mutex_(MUTEX_INIT)
    {
        mutex_init(&mutex_);
    }

    /// @brief Constructor with initial value
    /// @param initial_value The initial value (will be processed through all policies)
    explicit Parameter(const T& initial_value)
        : value_(initial_value), valid_(false), mutex_(MUTEX_INIT)
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
        }
        mutex_unlock(&mutex_);
        return valid_;
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
        bool result = false;

        mutex_lock(&mutex_);
        if constexpr (etl::is_same<T, float>::value) {
            message.set_float_value(value_);
            result = true;
        } else if constexpr (etl::is_same<T, double>::value) {
            message.set_double_value(value_);
            result = true;
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            message.set_int32_value(value_);
            result = true;
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            message.set_uint32_value(value_);
            result = true;
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            message.set_int64_value(value_);
            result = true;
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            message.set_uint64_value(value_);
            result = true;
        } else if constexpr (etl::is_same<T, bool>::value) {
            message.set_bool_value(value_);
            result = true;
        } else {
            // Unsupported type (should never happen due to Parameter static_assert)
            result = false;
        }
        mutex_unlock(&mutex_);

        return result;
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
            return set(new_value); // Apply validation (set() is already mutex-protected)
        }
        return false;
    }

  private:
    T value_;               ///< Parameter value
    bool valid_;            ///< Validity flag
    mutable mutex_t mutex_; ///< Mutex for thread-safe access
};

} // namespace parameter
} // namespace cogip

/// @}
