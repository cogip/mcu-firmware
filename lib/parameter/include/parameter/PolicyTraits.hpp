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

/// @brief Type trait to detect if Policy has on_commit(const T&) method
/// @tparam Policy The policy type to check
/// @tparam T The parameter value type
template <typename Policy, typename T, typename = void> struct has_on_commit : etl::false_type
{
};

/// @brief Specialization when Policy::on_commit(const T&) is valid
template <typename Policy, typename T>
struct has_on_commit<Policy, T,
                     etl::void_t<decltype(Policy::on_commit(etl::declval<const T&>()))>>
    : etl::true_type
{
};

/// @brief Type trait to detect if Policy has on_init(T&) method
/// @tparam Policy The policy type to check
/// @tparam T The parameter value type
template <typename Policy, typename T, typename = void> struct has_on_init : etl::false_type
{
};

/// @brief Specialization when Policy::on_init(T&) is valid
template <typename Policy, typename T>
struct has_on_init<Policy, T, etl::void_t<decltype(Policy::on_init(etl::declval<T&>()))>>
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

/// @brief Invoke on_commit on a single policy (post-validation persistence)
/// @tparam Policy The policy type
/// @tparam T The parameter value type
/// @param value The validated value to persist
template <typename Policy, typename T> void invoke_on_commit(const T& value)
{
    if constexpr (has_on_commit<Policy, T>::value) {
        Policy::on_commit(value);
    }
}

/// @brief Invoke on_init on a single policy (load from persistent storage)
/// @tparam Policy The policy type
/// @tparam T The parameter value type
/// @param value Reference to value (overwritten if policy loads a stored value)
/// @return true if value was loaded, false otherwise
template <typename Policy, typename T> bool invoke_on_init(T& value)
{
    if constexpr (has_on_init<Policy, T>::value) {
        return Policy::on_init(value);
    } else {
        return false;
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

/// @brief Fire on_commit on all policies (post-validation, fire-and-forget)
/// @tparam T The parameter value type
/// @tparam Policies The policy types to apply
/// @param value The validated value to persist
template <typename T, typename... Policies> void combined_on_commit(const T& value)
{
    (detail::invoke_on_commit<Policies, T>(value), ...);
}

/// @brief Try on_init on all policies using OR semantics (first load wins)
/// @tparam T The parameter value type
/// @tparam Policies The policy types to apply
/// @param value Reference to value (overwritten by first policy that loads successfully)
/// @return true if any policy loaded a value
template <typename T, typename... Policies> bool combined_on_init(T& value)
{
    return (detail::invoke_on_init<Policies, T>(value) || ...);
}

} // namespace parameter
} // namespace cogip

/// @}
