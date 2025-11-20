// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Type-erased and typed interfaces for polymorphic parameter manipulation
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

// Forward declaration
class PB_ParameterValue;

namespace cogip {

namespace parameter {

/// @brief Type-erased base class for all parameters
///
/// @note Provides conversion interface without type information.
///       Allows storing parameters of different types in heterogeneous collections.
class ParameterBase
{
  public:
    virtual ~ParameterBase() = default;

    /// @brief Convert parameter to protobuf PB_ParameterValue message
    /// @param message The PB_ParameterValue message to convert to
    /// @return true if conversion succeeded
    virtual bool pb_copy(PB_ParameterValue& message) const = 0;

    /// @brief Convert parameter from protobuf PB_ParameterValue message
    /// @param message The PB_ParameterValue message to convert from
    /// @return true if conversion succeeded
    virtual bool pb_read(const PB_ParameterValue& message) = 0;

    /// @brief Check if parameter holds valid value
    /// @return Status depending on the validation policy
    virtual bool isValid() const = 0;
};

/// @brief Typed interface for polymorphic parameter manipulation
/// @tparam T Value type (arithmetic types or bool)
///
/// @note Extends ParameterBase with type-specific operations (set/get).
///       Allows manipulating parameters with different policies via a common typed reference.
template <typename T> class ParameterInterface : public ParameterBase
{
  public:
    using value_type = T;

    virtual ~ParameterInterface() override = default;

    /// @brief Set parameter value with validation
    /// @return Value is correctly set or not. It depend on validation policy
    virtual bool set(const T& value) = 0;

    /// @brief Get current parameter value
    /// @return current value copy
    virtual T get() const = 0;
};

} // namespace parameter
} // namespace cogip

/// @}
