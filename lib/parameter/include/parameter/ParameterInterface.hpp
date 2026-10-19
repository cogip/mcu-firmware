// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Type-erased and typed interfaces for polymorphic parameter manipulation
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

// Forward declaration
class PB_ParameterValue;

namespace cogip {

namespace parameter {

/// @brief Type-erased base class for all parameters
///
/// @note Provides conversion and flash persistence without type information.
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
    /// @return Status depending on the range and the validation policies
    virtual bool isValid() const = 0;

    /// @brief Check whether the parameter value has changed since the last
    /// clear_changed() call.
    /// @return true if the value has been (re)set since the last clear.
    ///
    /// @details Intended for consumers that need to react to runtime updates
    /// (e.g. re-program a hardware register when a calibration scalar is
    /// changed). Newly constructed parameters are considered changed so the
    /// first poll picks up the initial value.
    virtual bool has_changed() const = 0;

    /// @brief Clear the "changed" flag. Call after handling a new value.
    /// Declared const because consumers typically keep read-only
    /// references to parameters they poll.
    virtual void clear_changed() const = 0;

    /// @brief Restore the compile-time default in memory
    /// @note Restores the initial value captured at construction, re-applies validation
    ///       policies and marks the parameter as changed so consumers pick it up.
    ///       Default implementation is a no-op (parameters without a default cannot reset).
    virtual void reset() {}

    /// @brief Load the value stored in flash under a key
    /// @note Without stored value, the current value is kept.
    ///
    /// @param key_hash Flash storage key
    /// @return false if the stored value is rejected (current value kept), true otherwise
    virtual bool load(uint32_t key_hash) = 0;

    /// @brief Store the current value in flash under a key
    /// @param key_hash Flash storage key
    virtual void store(uint32_t key_hash) const = 0;

    /// @brief Erase current value in flash under a key
    /// @param key_hash Flash storage key
    virtual void erase(uint32_t key_hash) const = 0;
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
