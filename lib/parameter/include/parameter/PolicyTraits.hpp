// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @file
/// @brief      Policy composition utilities for Parameter class
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include "etl/type_traits.h"

namespace cogip {
namespace parameter {

/// @brief Implementation details for policy trait detection
namespace detail {

/// @brief Type trait to detect if Policy has on_set(T&) method
/// @tparam Policy The policy type to check
/// @tparam T The parameter value type
template <typename Policy, typename T, typename = void> struct has_on_set : etl::false_type
{
};

/// @brief Specialization when Policy::on_set(T&) is valid
template <typename Policy, typename T>
struct has_on_set<Policy, T, etl::void_t<decltype(Policy::on_set(etl::declval<T&>()))>>
    : etl::true_type
{
};

/// @brief Invoke on_set on a single policy
/// @tparam Policy The policy type
/// @tparam T The parameter value type
/// @param value Reference to value (can be modified by policy)
/// @return Policy::on_set(value) if available, true otherwise
template <typename Policy, typename T> bool invoke_on_set(T& value)
{
    if constexpr (has_on_set<Policy, T>::value) {
        return Policy::on_set(value);
    } else {
        return true;
    }
}

} // namespace detail

/// @brief Combine on_set from all policies using AND semantics
/// @tparam T The parameter value type
/// @tparam Policies The policy types to apply
/// @param value Reference to value (can be modified by policies in sequence)
/// @return true only if ALL policies accept the value
/// @note Policies are applied left-to-right. Evaluation stops on first false (short-circuit).

template <typename T, typename... Policies> bool combined_on_set(T& value)
{
    return (detail::invoke_on_set<Policies, T>(value) && ...);
}

} // namespace parameter
} // namespace cogip

/// @}
