// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Policy-based parameter class template
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

#include "etl/type_traits.h"
#include "mutex.h"

#include "AccessPolicies.hpp"
#include "PB_ParameterCommands.hpp"
#include "ParameterInterface.hpp"
#include "ValidationPolicies.hpp"

namespace cogip {

namespace parameter {

/// @brief Policy-based parameter class template
/// @tparam T The C++ value type (arithmetic types or bool)
/// @tparam AccessPolicy Policy for read/write access control (default: ReadWrite)
/// @tparam ValidationPolicy Policy for value validation (default: NoValidation)
///
/// @note This class stores a value with validation and access policies applied.
///       Serializes to/from PB_ParameterValue messages (oneof value field).
template <typename T, typename AccessPolicy = ReadWrite, typename ValidationPolicy = NoValidation>
class Parameter : public ParameterInterface<T>
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
    /// @param initial_value The initial value (will be validated)
    explicit Parameter(const T& initial_value)
        : value_(initial_value), valid_(false), mutex_(MUTEX_INIT)
    {
        mutex_init(&mutex_);
        valid_ = ValidationPolicy::validate(value_);
    }

    /// @brief Set the parameter value with validation and access control
    /// @param value The new value to set
    /// @return true if value was set successfully (after validation and access check)
    ///
    /// @note This method applies the access and validation policies before updating the internal
    /// value.
    bool set(const T& value) override
    {
        // Check access policy first
        if (!AccessPolicy::canWrite()) {
            return false;
        }

        mutex_lock(&mutex_);
        valid_ = ValidationPolicy::validate(value);
        if (valid_) {
            value_ = value;
        }
        bool result = valid_;
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

    /// @brief Check if the parameter can be written to
    /// @return true if write operations are allowed, false otherwise
    bool canWrite() const override
    {
        return AccessPolicy::canWrite();
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
    /// message.
    ///
    /// @note Access policy and validation are applied to the converted value.
    bool pb_read(const PB_ParameterValue& message) override
    {
        // Check access policy first
        if (!AccessPolicy::canWrite()) {
            return false;
        }

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
