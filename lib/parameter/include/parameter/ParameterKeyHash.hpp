// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_parameter
/// @{
/// @brief      Compile-time FNV-1a hashing for parameter key identification
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once
#include <cstdint>

#include "etl/string_view.h"

namespace cogip {

namespace parameter {

/// @brief Constexpr FNV-1a hash implementation
/// @param str Pointer to current character
/// @param len Remaining length
/// @param hash Current hash value (FNV-1a offset basis: 0x811C9DC5)
/// @return The 32-bit hash value
///
/// @note This is a compile-time re-implementation of etl::fnv_1a_32.
///       ETL's implementation is not constexpr (constructor and methods are runtime-only),
///       preventing compile-time hash computation. This constexpr version uses the exact same
///       FNV-1a algorithm with identical constants:
///       - Offset basis: 0x811C9DC5
///       - Prime: 0x01000193
///       - Algorithm: hash = (hash ^ byte) * prime
///
///       This ensures compatibility: hash_key() produces the same results as etl::fnv_1a_32
///       but can be evaluated at compile-time for parameter key hashing.
constexpr uint32_t fnv1a_hash_bytes(const char* str, size_t len, uint32_t hash = 0x811C9DC5)
{
    return (len == 0) ? hash
                      : fnv1a_hash_bytes(str + 1, len - 1,
                                         (hash ^ static_cast<uint32_t>(*str)) * 0x01000193);
}

/// @brief Compile-time string hash using FNV-1a algorithm
/// @param key The string view to hash
/// @return The 32-bit hash value
///
/// @note Uses constexpr FNV-1a for compile-time hash computation.
///       Compatible with runtime hash from etl::fnv_1a_32.
constexpr uint32_t hash_key(etl::string_view key)
{
    return fnv1a_hash_bytes(key.data(), key.size());
}

/// @brief User-defined literal for compile-time key hashing
/// @param str The string literal
/// @param len The length of the string
/// @return The 32-bit hash value
///
/// @note Usage: "robot_speed"_key_hash
constexpr uint32_t operator"" _key_hash(const char* str, size_t len)
{
    return fnv1a_hash_bytes(str, len);
}

} // namespace parameter
} // namespace cogip

/// @}
