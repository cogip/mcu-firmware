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

} // namespace parameter_handler
} // namespace cogip
