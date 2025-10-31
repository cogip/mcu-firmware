// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file Parameter.hpp
/// @brief Policy-based parameter class for runtime configuration
///
/// This header provides a template-based parameter system that combines:
/// - Type-safe native types (all arithmetic types: int8_t, int16_t, int32_t, int64_t,
///   uint8_t, uint16_t, uint32_t, uint64_t, float, double, and bool)
/// - Compile-time behavior composition via policies
/// - Zero-overhead abstraction
/// - Serialization to ParameterValue protobuf messages
///
/// Usage example:
/// @code
/// using namespace cogip::parameter;
///
/// // Simple float parameter
/// Parameter<float> speed;
/// speed.set(25.5f);
/// float value = speed.get();
///
/// // Integer parameter with initial value
/// Parameter<int32_t> count{42};
///
/// // With validation bounds (bounds are integers, cast to float at runtime)
/// Parameter<float, WithBounds<-50, 150>> temp;
/// temp.set(200.0f);  // Validation fails, value remains unchanged
/// temp.set(25.0f);   // OK, within bounds
///
/// // Serialization to protobuf ParameterValue (enabled by default)
/// Parameter<float> wheel_diameter{80.5f};
/// ParameterValue param_value;
/// wheel_diameter.serialize(param_value);  // Fills param_value.float_value
///
/// // Deserialization from protobuf ParameterValue
/// Parameter<double> angle;
/// angle.deserialize(param_value);  // Reads param_value.double_value
///
/// @endcode

#pragma once

#include <cstdint>

#include "etl/type_traits.h"

#include "PB_ParameterCommands.hpp"
#include "ParameterInterface.hpp"
#include "ValidationPolicies.hpp"

namespace cogip {

namespace parameter {

/// @brief Policy-based parameter class template
/// @tparam T The C++ value type (arithmetic types: int8_t to int64_t, uint8_t to uint64_t, float,
/// double, or bool)
/// @tparam ValidationPolicy Policy for value validation (default: NoValidation)
///
/// This class stores a value with validation policy applied.
/// Serializes to/from ParameterValue messages (oneof value field).
/// Key management and protocol handling (Request/Response with key_hash) are
/// handled externally by a parameter registry or communication layer.
template <typename T, typename ValidationPolicy = NoValidation>
class Parameter : public ParameterInterface<T>
{
    // Compile-time type checking: only arithmetic types (integers, floats) and bool are supported
    static_assert(etl::is_arithmetic<T>::value || etl::is_same<T, bool>::value,
                  "Parameter<T> only supports arithmetic types (int, float, double) and bool");

  public:
    /// @brief Type alias for the value type
    using value_type = T;

    /// @brief Default constructor - initializes with invalid state
    Parameter() : value_{}, valid_(false) {}

    /// @brief Constructor with initial value
    /// @param initial_value The initial value (will be validated)
    explicit Parameter(const T& initial_value) : value_{}, valid_(false)
    {
        set(initial_value);
    }

    /// @brief Set the parameter value with validation
    /// @param value The new value to set
    /// @return true if value was set successfully (after validation)
    ///
    /// This method applies the validation policy and updates the internal value.
    bool set(const T& value) override
    {
        T validated_value = value_;  // Initialize with current value
        valid_ = ValidationPolicy::validate(value, validated_value);
        if (valid_) {
            value_ = validated_value;
        }
        return valid_;
    }

    /// @brief Get the current parameter value
    /// @return The current value
    ///
    /// @note This returns the value even if the parameter is marked invalid.
    ///       Use isValid() to check validity before using the value.
    T get() const override
    {
        return value_;
    }

    /// @brief Check if the parameter holds a valid value
    /// @return true if the value is valid, false otherwise
    bool isValid() const override
    {
        return valid_;
    }

    /// @brief Serialize parameter value to protobuf ParameterValue message
    /// @param message The ParameterValue message to serialize into
    /// @return true if serialization succeeded, false otherwise
    ///
    /// Uses compile-time type dispatch to set the correct oneof field in ParameterValue message.
    bool serialize(ParameterValue& message) const override
    {
        if constexpr (etl::is_same<T, float>::value) {
            message.set_float_value(value_);
            return true;
        } else if constexpr (etl::is_same<T, double>::value) {
            message.set_double_value(value_);
            return true;
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            message.set_int32_value(value_);
            return true;
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            message.set_uint32_value(value_);
            return true;
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            message.set_int64_value(value_);
            return true;
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            message.set_uint64_value(value_);
            return true;
        } else if constexpr (etl::is_same<T, bool>::value) {
            message.set_bool_value(value_);
            return true;
        } else {
            // Unsupported type (should never happen due to Parameter static_assert)
            return false;
        }
    }

    /// @brief Deserialize parameter value from protobuf ParameterValue message
    /// @param message The ParameterValue message to deserialize from
    /// @return true if deserialization succeeded, false otherwise
    ///
    /// Uses compile-time type dispatch to get the correct oneof field from ParameterValue message.
    /// Validation is applied to the deserialized value.
    bool deserialize(const ParameterValue& message) override
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
            return set(new_value); // Apply validation
        }
        return false;
    }

  private:
    T value_;    ///< Parameter value
    bool valid_; ///< Validity flag
};

} // namespace parameter

} // namespace cogip
