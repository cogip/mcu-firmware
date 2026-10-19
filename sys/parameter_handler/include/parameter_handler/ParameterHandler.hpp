// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterHandler.hpp
/// @brief Generic parameter handler for CAN protobuf announce/list/get/set/reset requests

#pragma once

// Standard includes
#include <cstddef>
#include <cstdint>

// ETL includes
#include "etl/optional.h"

// Project includes
#include "canpb/ReadBuffer.hpp"
#include "parameter_handler/ParameterEntry.hpp"

// Protobuf messages
#include "PB_ParameterRegistry.hpp"

namespace cogip {
namespace parameter_handler {

/// Announce of a registry, see ParameterHandler::announce()
using ParameterGroup = PB_ParameterGroup<name_length_max>;

/// Definition of a registry parameter, see ParameterHandler::handle_definition()
using ParameterDefinition = PB_ParameterDefinition<name_length_max>;

/// @brief Parameters of a board, announced, read, written and reset through CAN protobuf requests
/// @details Answers only for the parameters of its registry, so that each board on the CAN bus
///          answers for its own. For a persistent parameter, load() restores the stored
///          value, a set request stores the new value and a reset request erases it.
class ParameterHandler
{
  public:
    /// @brief Construct handler with a name and a registry
    /// @tparam N Size of the name, null terminator included
    /// @param name Registry name, its FNV-1a hash addresses the list requests
    /// @param registry Parameters of the board
    template <size_t N>
    ParameterHandler(const char (&name)[N], ParameterRegistry registry)
        : name_(name), name_hash_(utils::fnv1a_hash_bytes(name, N - 1)), registry_(registry)
    {
        // Name field, count and registry_hash varints of PB_ParameterGroup
        static_assert(2 + (N - 1) + 6 + 6 <= canpb_message_size_max,
                      "Registry name too long for a canpb announce");
    }

    /// @brief Load the parameters stored in flash
    /// @note A parameter whose stored value is rejected keeps its default value.
    void load();

    /// @brief Get the announce of the registry
    /// @return Registry name, number of parameters and hash of their key hashes
    /// @note The receiver checks it got every key hash of the list by hashing them the same way.
    ParameterGroup announce() const;

    /// @brief Handle parameter list request
    /// @param buffer CAN read buffer containing serialized PB_ParameterListRequest
    /// @return true if the request targets this registry, then send list_response() of each
    ///         parameter, false otherwise
    bool handle_list(canpb::ReadBuffer& buffer) const;

    /// @brief Get the number of parameters in the registry
    /// @return Number of parameters
    size_t size() const
    {
        return registry_.size();
    }

    /// @brief Get the list response of a parameter
    /// @param index Index of the parameter in the registry, lower than size()
    /// @return Key hash of the parameter
    PB_ParameterListResponse list_response(size_t index) const;

    /// @brief Handle parameter definition request
    /// @param buffer CAN read buffer containing serialized PB_ParameterDefinitionRequest
    /// @return Optional response with the name and accepted range of the parameter, empty if
    ///         parameter not found locally
    /// @note Returns empty optional if parameter not in this board's registry, allowing other
    ///       boards on the CAN bus to respond instead. Only the board that owns the parameter
    ///       should send a response.
    etl::optional<ParameterDefinition> handle_definition(canpb::ReadBuffer& buffer) const;

    /// @brief Handle parameter get request
    /// @param buffer CAN read buffer containing serialized PB_ParameterGetRequest
    /// @return Optional response message with parameter value, empty if parameter not found locally
    /// @note Returns empty optional if parameter not in this board's registry, allowing other
    ///       boards on the CAN bus to respond instead. Only the board that owns the parameter
    ///       should send a response.
    etl::optional<PB_ParameterGetResponse> handle_get(canpb::ReadBuffer& buffer);

    /// @brief Handle parameter set request
    /// @param buffer CAN read buffer containing serialized PB_ParameterSetRequest
    /// @return Optional response message with operation status, empty if parameter not found
    /// locally
    /// @note Returns empty optional if parameter not in this board's registry, allowing other
    ///       boards on the CAN bus to respond instead. Only the board that owns the parameter
    ///       should send a response.
    etl::optional<PB_ParameterSetResponse> handle_set(canpb::ReadBuffer& buffer);

    /// @brief Handle parameter reset request
    /// @param buffer CAN read buffer containing serialized PB_ParameterResetRequest
    /// @return Optional response with operation status, empty if parameter not found locally
    /// @note Returns empty optional if parameter not in this board's registry, allowing other
    ///       boards on the CAN bus to respond instead. Only the board that owns the parameter
    ///       should send a response.
    etl::optional<PB_ParameterResetResponse> handle_reset(canpb::ReadBuffer& buffer);

  private:
    /// @brief Find a parameter by key hash
    /// @param key_hash FNV-1a hash of the parameter name
    /// @return Registry entry, nullptr if the parameter is not in the registry
    const ParameterEntry* find(uint32_t key_hash) const;

    const char* name_;           ///< Registry name
    uint32_t name_hash_;         ///< FNV-1a hash of the registry name
    ParameterRegistry registry_; ///< Parameters of the board
};

} // namespace parameter_handler
} // namespace cogip
