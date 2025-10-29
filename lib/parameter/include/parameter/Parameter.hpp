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
/// - CAN bus integration for get/set operations
///
/// Usage example:
/// @code
/// // Simple float parameter
/// Parameter<float> speed;
/// speed.set(25.5f);
/// float value = speed.get();
///
/// // Integer parameter
/// Parameter<int32_t> count;
/// count.set(42);
///
/// // With validation bounds
/// using TempParam = Parameter<float, WithBounds<-50.0f, 150.0f>>;
/// TempParam temp;
/// temp.set(200.0f);  // Auto-clamped to 150.0f
///
/// // With protobuf serialization
/// using SerializableParam = Parameter<float, NoValidation, NoStorage, WithProtobuf>;
/// SerializableParam wheel_diameter;
/// wheel_diameter.set(80.5f);
/// ParameterSetRequest request;
/// wheel_diameter.serialize(request);  // Serializes to float_value field
///
/// // Polymorphic usage via ParameterInterface<T> interface
/// Parameter<float> speed_param{25.0f};
/// Parameter<float, WithBounds<0.0f, 100.0f>> humidity_param{60.0f};
///
/// // Manipulate via interface reference
/// ParameterInterface<float>& param_ref1 = speed_param;
/// ParameterInterface<float>& param_ref2 = humidity_param;
/// param_ref1.set(30.0f);
/// bool valid = param_ref2.isValid();
///
/// // Heterogeneous collections
/// etl::array<ParameterInterface<float>*, 2> params = {&speed_param, &humidity_param};
/// for (auto* p : params) {
///     if (p->isValid()) {
///         float val = p->get();
///         // Process parameter value
///     }
/// }
/// @endcode

#pragma once

#include <cstdint>

#include "etl/type_traits.h"

#include "ParameterInterface.hpp"
#include "SerializationPolicies.hpp"
#include "StoragePolicies.hpp"
#include "ValidationPolicies.hpp"

namespace cogip {

namespace parameter {

/// @brief Policy-based parameter class template
/// @tparam T The C++ value type (arithmetic types: int8_t to int64_t, uint8_t to uint64_t, float,
/// double, or bool)
/// @tparam ValidationPolicy Policy for value validation (default: NoValidation)
/// @tparam StoragePolicy Policy for persistence (default: NoStorage)
/// @tparam SerializationPolicy Policy for serialization (default: NoSerialization)
///
/// This class stores a value with validation, storage, and serialization policies applied.
/// Serialization for CAN communication is handled via ParameterSetRequest/Response
/// messages defined in PB_ParameterCommands.proto.
template <typename T, typename ValidationPolicy = NoValidation, typename StoragePolicy = NoStorage,
          typename SerializationPolicy = NoSerialization>
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
        T validated_value;
        valid_ = ValidationPolicy::validate(value, validated_value);
        value_ = validated_value;
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

    /// @brief Load parameter from persistent storage
    /// @return true if loaded successfully, false otherwise
    ///
    /// Uses the storage policy to load the value from non-volatile memory.
    /// If successful, the loaded value is validated and set.
    bool load()
    {
        T loaded_value;
        if (StoragePolicy::load(loaded_value)) {
            return set(loaded_value);
        }
        return false;
    }

    /// @brief Store parameter to persistent storage
    /// @return true if stored successfully, false otherwise
    ///
    /// Uses the storage policy to persist the current value to non-volatile memory.
    bool store() const
    {
        if (isValid()) {
            return StoragePolicy::store(get());
        }
        return false;
    }

    /// @brief Serialize parameter value to protobuf message
    /// @tparam MsgType The protobuf message type (e.g., ParameterSetRequest)
    /// @param message The protobuf message to serialize into
    /// @return true if serialization succeeded, false otherwise
    ///
    /// Uses the serialization policy to serialize the current value into the appropriate
    /// oneof field of the protobuf message.
    template <typename MsgType> bool serialize(MsgType& message) const
    {
        return SerializationPolicy::template serialize<T, MsgType>(value_, message);
    }

    /// @brief Deserialize parameter value from protobuf message
    /// @tparam MsgType The protobuf message type (e.g., ParameterGetResponse)
    /// @param message The protobuf message to deserialize from
    /// @return true if deserialization succeeded, false otherwise
    ///
    /// Uses the serialization policy to deserialize the value from the appropriate
    /// oneof field of the protobuf message and update the parameter.
    /// Validation is applied to the deserialized value.
    template <typename MsgType> bool deserialize(const MsgType& message)
    {
        T new_value;
        if (SerializationPolicy::template deserialize<T, MsgType>(message, new_value)) {
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
