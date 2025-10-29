// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file SerializationPolicies.hpp
/// @brief Serialization policies for parameters
///
/// These policies provide compile-time behavior composition for parameter
/// serialization/deserialization to/from various formats (protobuf, JSON, etc.).

#pragma once
#include <cstdint>

#include "etl/type_traits.h"

namespace cogip {

namespace parameter {

/// @brief No serialization policy - parameters cannot be serialized
/// @details This is the default policy that provides no serialization.
struct NoSerialization
{
    /// @brief Serialize parameter to protobuf message (no-op)
    /// @tparam T The parameter value type
    /// @tparam MsgType The protobuf message type
    /// @param value The parameter value
    /// @param message The message to serialize into (unused)
    /// @return false Always returns false (no serialization available)
    template <typename T, typename MsgType>
    static bool serialize(const T& /* value */, MsgType& /* message */)
    {
        return false;
    }

    /// @brief Deserialize parameter from protobuf message (no-op)
    /// @tparam T The parameter value type
    /// @tparam MsgType The protobuf message type
    /// @param message The message to deserialize from (unused)
    /// @param value The parameter value to deserialize into (unchanged)
    /// @return false Always returns false (no deserialization available)
    template <typename T, typename MsgType>
    static bool deserialize(const MsgType& /* message */, T& /* value */)
    {
        return false;
    }
};

/// @brief Protobuf serialization policy - enables protobuf serialization
/// @details This policy implements type-specific serialization/deserialization
///          for protobuf messages with oneof fields (ParameterSetRequest/ParameterGetResponse).
struct WithProtobuf
{
    /// @brief Serialize parameter value to protobuf message
    /// @tparam T The parameter value type
    /// @tparam MsgType The protobuf message type (e.g., ParameterSetRequest)
    /// @param value The parameter value to serialize
    /// @param message The protobuf message to serialize into
    /// @return true if serialization succeeded
    ///
    /// Uses compile-time type dispatch to set the correct oneof field.
    template <typename T, typename MsgType> static bool serialize(const T& value, MsgType& message)
    {
        if constexpr (etl::is_same<T, float>::value) {
            message.set_float_value(value);
            return true;
        } else if constexpr (etl::is_same<T, double>::value) {
            message.set_double_value(value);
            return true;
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            message.set_int32_value(value);
            return true;
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            message.set_uint32_value(value);
            return true;
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            message.set_int64_value(value);
            return true;
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            message.set_uint64_value(value);
            return true;
        } else if constexpr (etl::is_same<T, bool>::value) {
            message.set_bool_value(value);
            return true;
        } else {
            // Unsupported type (should never happen due to Parameter static_assert)
            return false;
        }
    }

    /// @brief Deserialize parameter value from protobuf message
    /// @tparam T The parameter value type
    /// @tparam MsgType The protobuf message type (e.g., ParameterGetResponse)
    /// @param message The protobuf message to deserialize from
    /// @param value The parameter value to deserialize into
    /// @return true if deserialization succeeded
    ///
    /// Uses compile-time type dispatch to get the correct oneof field.
    template <typename T, typename MsgType>
    static bool deserialize(const MsgType& message, T& value)
    {
        if constexpr (etl::is_same<T, float>::value) {
            value = message.get_float_value();
            return true;
        } else if constexpr (etl::is_same<T, double>::value) {
            value = message.get_double_value();
            return true;
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            value = message.get_int32_value();
            return true;
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            value = message.get_uint32_value();
            return true;
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            value = message.get_int64_value();
            return true;
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            value = message.get_uint64_value();
            return true;
        } else if constexpr (etl::is_same<T, bool>::value) {
            value = message.get_bool_value();
            return true;
        } else {
            // Unsupported type (should never happen due to Parameter static_assert)
            return false;
        }
    }
};

} // namespace parameter

} // namespace cogip
