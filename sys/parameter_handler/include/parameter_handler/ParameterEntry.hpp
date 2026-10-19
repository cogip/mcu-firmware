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

// Project includes
#include "KeyHash.hpp"
#include "parameter/ParameterInterface.hpp"

namespace cogip {
namespace parameter_handler {

/// @brief Parameter available for read/write through canpb, identified by its name
/// @note The hash of the name is also the flash storage key of a parameter stored in flash.
struct ParameterEntry
{
    static constexpr bool Store = true;

    /// @brief Register a parameter under a name
    /// @tparam N Size of the name, null terminator included
    /// @param name Parameter name, its FNV-1a hash is the key hash and the flash storage key
    /// @param parameter Parameter object
    /// @param storage Storage of the parameter value
    template <size_t N>
    constexpr ParameterEntry(const char (&name)[N], cogip::parameter::ParameterBase& parameter,
                             bool persistent = false)
        : name(name), key_hash(utils::fnv1a_hash_bytes(name, N - 1)), parameter(parameter),
          persistent_(persistent)
    {
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
