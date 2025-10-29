// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterInterface.hpp
/// @brief Minimal interface for polymorphic parameter manipulation
///
/// This header provides a minimal interface that allows manipulating parameters
/// with different policies via a common reference. Only essential operations
/// (set/get/isValid) are virtual to minimize overhead.
///
/// Usage example:
/// @code
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

namespace cogip {

namespace parameter {

/// @brief Minimal interface for polymorphic parameter manipulation
/// @tparam T Value type (arithmetic types or bool)
///
/// Allows manipulating parameters with different policies via a common reference.
/// Only essential operations (set/get/isValid) are virtual to minimize overhead.
///
/// @note serialize/deserialize methods are not virtual
template <typename T> class ParameterInterface
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
