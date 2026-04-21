// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterDescriptor.hpp
/// @brief Runtime metadata describing one parameter exposed to the CAN registry.

#pragma once

// Standard includes
#include <cinttypes>
#include <cstdint>

// Project includes
#include "parameter/Parameter.hpp"
#include "parameter/ParameterInterface.hpp"

// Protobuf messages (for PB_ParameterType / PB_ParameterTag enums)
#include "PB_ParameterCommands.hpp"

namespace cogip {
namespace parameter_handler {

/// @brief Scalar bound value for a parameter descriptor
///
/// @note PB_ParameterValue is not trivially constructible from a static
///       initializer (generated oneof class). The descriptor stores raw
///       scalars and the announce handler fills PB_ParameterValue at send
///       time based on `type`.
struct ParameterBoundValue
{
    PB_ParameterType type;
    union {
        float    float_value;
        double   double_value;
        int32_t  int32_value;
        uint32_t uint32_value;
        int64_t  int64_value;
        uint64_t uint64_value;
        bool     bool_value;
    };

    /// @brief Emit the bound as a PB_ParameterValue oneof field
    /// @return true on success, false if `type` is unknown
    bool pb_copy(PB_ParameterValue& message) const
    {
        switch (type) {
        case PB_ParameterType::PARAM_TYPE_FLOAT:  message.set_float_value(float_value);   return true;
        case PB_ParameterType::PARAM_TYPE_DOUBLE: message.set_double_value(double_value); return true;
        case PB_ParameterType::PARAM_TYPE_INT32:  message.set_int32_value(int32_value);   return true;
        case PB_ParameterType::PARAM_TYPE_UINT32: message.set_uint32_value(uint32_value); return true;
        case PB_ParameterType::PARAM_TYPE_INT64:  message.set_int64_value(int64_value);   return true;
        case PB_ParameterType::PARAM_TYPE_UINT64: message.set_uint64_value(uint64_value); return true;
        case PB_ParameterType::PARAM_TYPE_BOOL:   message.set_bool_value(bool_value);     return true;
        }
        return false;
    }
};

/// @brief Runtime metadata describing a parameter registered on a board.
///
/// @note The registry is a flat etl::vector of descriptors. All fields are
///       populated manually for v1; a follow-up will derive `type`,
///       `read_only`, `has_bounds`, `min_value` and `max_value` from
///       parameter policy traits (cf. lib/parameter/PolicyTraits.hpp).
/// @note `param` is a pointer (never null in a populated descriptor) instead
///       of a reference so the descriptor is trivially copyable and can be
///       stored in container types that need copy-assignment.
struct ParameterDescriptor
{
    uint32_t                   key_hash;       ///< FNV-1a hash of the parameter name
    const char*                name;           ///< Flash-resident literal, ≤ 36 chars
    PB_ParameterType           type;           ///< Scalar type of the parameter
    uint32_t                   tags_bitmask;   ///< OR of (1u << PB_ParameterTag::X)
    bool                       read_only;      ///< true if ReadOnly policy is set
    bool                       has_bounds;     ///< true if min/max are meaningful
    ParameterBoundValue        min_value;      ///< lower bound (only if has_bounds)
    ParameterBoundValue        max_value;      ///< upper bound (only if has_bounds)
    parameter::ParameterBase*  param;          ///< live parameter, non-null
};

/// @brief Build the tags bitmask from a list of PB_ParameterTag values
///
/// @note Usage: `param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
///                              PB_ParameterTag::PARAM_TAG_LINEAR)`
template <typename... Tags>
constexpr uint32_t param_tag_bits(Tags... tags)
{
    return (0u | ... | (1u << static_cast<uint32_t>(tags)));
}

namespace detail {

/// @brief Build a ParameterBoundValue from the scalar bound of a Parameter<T, ...>.
///
/// @note `BoundValue` is the compile-time NTTP extracted by
///       `bounds_min_v` / `bounds_max_v`; `ValueT` is the underlying C++ type
///       (float, int32_t, ...) so we pick the right union slot.
template <typename ValueT, auto BoundValue> constexpr ParameterBoundValue make_bound()
{
    ParameterBoundValue bound{};
    bound.type = parameter::pb_type_of<ValueT>();
    if constexpr (std::is_same_v<ValueT, float>) {
        bound.float_value = static_cast<float>(BoundValue);
    } else if constexpr (std::is_same_v<ValueT, double>) {
        bound.double_value = static_cast<double>(BoundValue);
    } else if constexpr (std::is_same_v<ValueT, int32_t>) {
        bound.int32_value = static_cast<int32_t>(BoundValue);
    } else if constexpr (std::is_same_v<ValueT, uint32_t>) {
        bound.uint32_value = static_cast<uint32_t>(BoundValue);
    } else if constexpr (std::is_same_v<ValueT, int64_t>) {
        bound.int64_value = static_cast<int64_t>(BoundValue);
    } else if constexpr (std::is_same_v<ValueT, uint64_t>) {
        bound.uint64_value = static_cast<uint64_t>(BoundValue);
    } else if constexpr (std::is_same_v<ValueT, bool>) {
        bound.bool_value = static_cast<bool>(BoundValue);
    }
    return bound;
}

} // namespace detail

/// @brief Build a ParameterDescriptor from a Parameter<T, Policies...>.
///
/// @details All fields except `key_hash`, `name` and `tags_bitmask` are
///          derived from the parameter's policy pack via `parameter_traits`:
///          the PB type comes from `T`, `read_only` from the presence of
///          `ReadOnly`, and bounds from the first `Clamp` / `WithBounds` in
///          the pack.
template <typename ParameterT>
constexpr ParameterDescriptor make_descriptor(uint32_t key_hash, const char* name,
                                              uint32_t tags_bitmask, ParameterT& param)
{
    using traits = parameter::parameter_traits<ParameterT>;
    using value_type = typename traits::value_type;

    ParameterDescriptor desc{};
    desc.key_hash = key_hash;
    desc.name = name;
    desc.type = traits::pb_type;
    desc.tags_bitmask = tags_bitmask;
    desc.read_only = traits::read_only;
    desc.has_bounds = traits::has_bounds;
    if constexpr (traits::has_bounds) {
        desc.min_value = detail::make_bound<value_type, traits::min_value>();
        desc.max_value = detail::make_bound<value_type, traits::max_value>();
    }
    desc.param = &param;
    return desc;
}

/// @brief Shorthand used at registration sites.
///
/// @note Usage: `DECLARE_PARAM(key_hash, "name", tags_bitmask, param_object)`
///       All metadata except tags is deduced from the Parameter<> policy pack.
#define DECLARE_PARAM(key_hash, name, tags_bitmask, param_ref)                                     \
    ::cogip::parameter_handler::make_descriptor((key_hash), (name), (tags_bitmask), (param_ref))

} // namespace parameter_handler
} // namespace cogip
