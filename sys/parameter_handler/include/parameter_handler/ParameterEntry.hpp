// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterEntry.hpp
/// @brief Named parameter of a board registry

#pragma once

// Standard includes
#include <cstddef>
#include <cstdint>

// ETL includes
#include "etl/span.h"
#include "etl/type_traits.h"

// Project includes
#include "KeyHash.hpp"
#include "parameter/ParameterInterface.hpp"

namespace cogip {
namespace parameter_handler {

/// Largest protobuf message canpb sends: its 60 base64 bytes fit a 64-byte CAN FD frame
constexpr size_t canpb_message_size_max = 45;

/// Longest name a canpb message can carry, after the tag and length of the field
constexpr size_t name_length_max = canpb_message_size_max - 2;

/// @brief Largest size of a PB_ParameterValue field holding a T
/// @tparam T Parameter value type
/// @return Size in bytes of the field tag and length, the value tag and the largest value
template <typename T> constexpr size_t pb_value_size_max()
{
    constexpr size_t tags_size = 3; // Field tag, field length and value tag

    if constexpr (etl::is_same<T, bool>::value) {
        return tags_size + 1;
    } else if constexpr (etl::is_floating_point<T>::value) {
        return tags_size + sizeof(T);
    } else if constexpr (etl::is_same<T, uint32_t>::value) {
        return tags_size + 5;
    } else {
        // Varint of a 64-bit or a negative 32-bit integer
        return tags_size + 10;
    }
}

/// @brief Parameter available for read/write through canpb, identified by its name
/// @note The hash of the name is also the flash storage key of a parameter stored in flash.
struct ParameterEntry
{
    static constexpr bool Store = true;

    /// @brief Register a parameter under a name
    /// @tparam N Size of the name, null terminator included
    /// @tparam T Parameter value type
    /// @param name Parameter name, its FNV-1a hash is the key hash and the flash storage key
    /// @param parameter Parameter object
    /// @param persistent Store the parameter value in flash
    /// @note The name must fit a PB_ParameterDefinition with the largest bounds of T.
    template <size_t N, typename T>
    constexpr ParameterEntry(const char (&name)[N],
                             cogip::parameter::ParameterInterface<T>& parameter,
                             bool persistent = false)
        : name(name), key_hash(utils::fnv1a_hash_bytes(name, N - 1)), parameter(parameter),
          persistent_(persistent)
    {
        static_assert(2 + (N - 1) + 2 * pb_value_size_max<T>() <= canpb_message_size_max,
                      "Parameter name too long for a canpb definition");
    }

    /// @brief Load the parameter from flash
    /// @return false if the stored value is rejected (default value kept), true otherwise
    bool load() const
    {
        if (!persistent_) {
            return true;
        }

        return parameter.load(key_hash);
    }

    /// @brief Store the parameter in flash
    void store() const
    {
        if (!persistent_) {
            return;
        }

        parameter.store(key_hash);
    }

    /// @brief Erase the parameter from flash
    void erase() const
    {
        if (!persistent_) {
            return;
        }

        parameter.erase(key_hash);
    }

    const char* name;                           ///< Parameter name
    const uint32_t key_hash;                    ///< FNV-1a hash of the name
    cogip::parameter::ParameterBase& parameter; ///< Parameter object

  private:
    const bool persistent_; ///< Storage of the parameter value
};

/// Parameters of a board, in declaration order
using ParameterRegistry = etl::span<const ParameterEntry>;

} // namespace parameter_handler
} // namespace cogip
