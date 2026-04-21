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

#include <cstdint>

#include "etl/type_traits.h"

#include "AccessPolicies.hpp"
#include "PB_ParameterCommands.hpp"

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

/// @brief Type trait to detect if Policy has on_pb_read(T&) method
template <typename Policy, typename T, typename = void> struct has_on_pb_read : etl::false_type
{
};

template <typename Policy, typename T>
struct has_on_pb_read<Policy, T, etl::void_t<decltype(Policy::on_pb_read(etl::declval<T&>()))>>
    : etl::true_type
{
};

/// @brief Type trait to detect if Policy has on_pb_copy(T&) method
template <typename Policy, typename T, typename = void> struct has_on_pb_copy : etl::false_type
{
};

template <typename Policy, typename T>
struct has_on_pb_copy<Policy, T, etl::void_t<decltype(Policy::on_pb_copy(etl::declval<T&>()))>>
    : etl::true_type
{
};

/// @brief Invoke on_pb_read on a single policy (convert user units to internal units)
template <typename Policy, typename T> void invoke_on_pb_read(T& value)
{
    if constexpr (has_on_pb_read<Policy, T>::value) {
        Policy::on_pb_read(value);
    }
}

/// @brief Invoke on_pb_copy on a single policy (convert internal units to user units)
template <typename Policy, typename T> void invoke_on_pb_copy(T& value)
{
    if constexpr (has_on_pb_copy<Policy, T>::value) {
        Policy::on_pb_copy(value);
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

/// @brief Apply on_pb_read from all policies (convert user units to internal units)
/// @tparam T The parameter value type
/// @tparam Policies The policy types to apply
/// @param value Reference to value (modified by conversion policies)
template <typename T, typename... Policies> void combined_on_pb_read(T& value)
{
    (detail::invoke_on_pb_read<Policies, T>(value), ...);
}

/// @brief Apply on_pb_copy from all policies (convert internal units to user units)
/// @tparam T The parameter value type
/// @tparam Policies The policy types to apply
/// @param value Reference to value (modified by conversion policies)
template <typename T, typename... Policies> void combined_on_pb_copy(T& value)
{
    (detail::invoke_on_pb_copy<Policies, T>(value), ...);
}

// ---------------------------------------------------------------------------
// Descriptor traits
// ---------------------------------------------------------------------------
// Compile-time helpers that let the parameter handler registry auto-derive
// the per-parameter metadata (PB type, read-only flag, bounds presence and
// values) from a Parameter<T, Policies...>, so the registration site only
// has to name the parameter, attach tags, and keep the symbol reference.

/// @brief Map a C++ scalar type to its PB_ParameterType enum value.
template <typename T> constexpr PB_ParameterType pb_type_of()
{
    if constexpr (etl::is_same<T, float>::value) {
        return PB_ParameterType::PARAM_TYPE_FLOAT;
    } else if constexpr (etl::is_same<T, double>::value) {
        return PB_ParameterType::PARAM_TYPE_DOUBLE;
    } else if constexpr (etl::is_same<T, int32_t>::value) {
        return PB_ParameterType::PARAM_TYPE_INT32;
    } else if constexpr (etl::is_same<T, uint32_t>::value) {
        return PB_ParameterType::PARAM_TYPE_UINT32;
    } else if constexpr (etl::is_same<T, int64_t>::value) {
        return PB_ParameterType::PARAM_TYPE_INT64;
    } else if constexpr (etl::is_same<T, uint64_t>::value) {
        return PB_ParameterType::PARAM_TYPE_UINT64;
    } else if constexpr (etl::is_same<T, bool>::value) {
        return PB_ParameterType::PARAM_TYPE_BOOL;
    } else {
        static_assert(etl::is_same<T, void>::value, "Unsupported parameter scalar type");
        return PB_ParameterType::PARAM_TYPE_FLOAT;
    }
}

/// @brief True iff at least one policy in the pack is `ReadOnly`.
template <typename... Policies>
constexpr bool is_read_only_v = (etl::is_same<Policies, ReadOnly>::value || ...);

namespace detail {

/// @brief Detects the `is_bounds_policy` static marker on Clamp / WithBounds.
template <typename Policy, typename = void> struct policy_is_bounds : etl::false_type
{
};

template <typename Policy>
struct policy_is_bounds<Policy, etl::void_t<decltype(Policy::is_bounds_policy)>>
    : etl::bool_constant<Policy::is_bounds_policy>
{
};

/// @brief Pick the first bounds-carrying policy in the pack, or `void` if none.
template <typename... Policies> struct first_bounds_policy;

template <> struct first_bounds_policy<>
{
    using type = void;
};

template <typename Head, typename... Tail> struct first_bounds_policy<Head, Tail...>
{
    using type =
        etl::conditional_t<policy_is_bounds<Head>::value, Head, typename first_bounds_policy<Tail...>::type>;
};

} // namespace detail

/// @brief True iff any policy in the pack declares compile-time bounds.
template <typename... Policies>
constexpr bool has_bounds_v = (detail::policy_is_bounds<Policies>::value || ...);

/// @brief Compile-time lower bound across the pack (0 when has_bounds_v is false).
template <typename... Policies> constexpr auto bounds_min_v = []() {
    using BoundsPolicy = typename detail::first_bounds_policy<Policies...>::type;
    if constexpr (etl::is_same<BoundsPolicy, void>::value) {
        return 0;
    } else {
        return BoundsPolicy::min_bound;
    }
}();

/// @brief Compile-time upper bound across the pack (0 when has_bounds_v is false).
template <typename... Policies> constexpr auto bounds_max_v = []() {
    using BoundsPolicy = typename detail::first_bounds_policy<Policies...>::type;
    if constexpr (etl::is_same<BoundsPolicy, void>::value) {
        return 0;
    } else {
        return BoundsPolicy::max_bound;
    }
}();

} // namespace parameter
} // namespace cogip

/// @}
