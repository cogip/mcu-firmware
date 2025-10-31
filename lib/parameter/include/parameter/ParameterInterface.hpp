// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterInterface.hpp
/// @brief Minimal interface for polymorphic parameter manipulation
///
/// This header provides a type-erased base class and typed interface for parameters.
/// This allows storing parameters of different types in a single heterogeneous collection.
///
/// Usage example:
/// @code
/// Parameter<float> speed_param{25.0f};
/// Parameter<int32_t> count_param{42};
///
/// // Type-specific manipulation via ParameterInterface<T>
/// ParameterInterface<float>& float_ref = speed_param;
/// float_ref.set(30.0f);
///
/// // Type-erased manipulation via ParameterBase (for heterogeneous collections)
/// etl::map<uint32_t, ParameterBase*> registry = {
///     {0x1001, &speed_param},
///     {0x1002, &count_param}  // Different types in same map!
/// };
///
/// for (auto& entry : registry) {
///     ParameterValue msg;
///     entry.second->serialize(msg);  // Works for any type
/// }
/// @endcode

#pragma once

// Forward declaration
class ParameterValue;

namespace cogip {

namespace parameter {

/// @brief Type-erased base class for all parameters
///
/// Provides serialization interface without type information.
/// Allows storing parameters of different types in heterogeneous collections.
class ParameterBase
{
  public:
    virtual ~ParameterBase() = default;

    /// @brief Serialize parameter to protobuf ParameterValue message
    /// @param message The ParameterValue message to serialize into
    /// @return true if serialization succeeded
    virtual bool serialize(ParameterValue& message) const = 0;

    /// @brief Deserialize parameter from protobuf ParameterValue message
    /// @param message The ParameterValue message to deserialize from
    /// @return true if deserialization succeeded
    virtual bool deserialize(const ParameterValue& message) = 0;
};

/// @brief Typed interface for polymorphic parameter manipulation
/// @tparam T Value type (arithmetic types or bool)
///
/// Extends ParameterBase with type-specific operations (set/get/isValid).
/// Allows manipulating parameters with different policies via a common typed reference.
template <typename T> class ParameterInterface : public ParameterBase
{
  public:
    using value_type = T;

    virtual ~ParameterInterface() = default;

    /// @brief Set parameter value with validation
    virtual bool set(const T& value) = 0;

    /// @brief Get current parameter value
    virtual T get() const = 0;

    /// @brief Check if parameter holds valid value
    virtual bool isValid() const = 0;
};

} // namespace parameter

} // namespace cogip
