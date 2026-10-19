// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterHandler.hpp
/// @brief Generic parameter handler for CAN protobuf get/set/reset requests

#pragma once

// Standard includes
#include <cstdint>

// ETL includes
#include "etl/optional.h"

// Project includes
#include "canpb/ReadBuffer.hpp"
#include "parameter_handler/ParameterEntry.hpp"

// Protobuf messages
#include "PB_ParameterCommands.hpp"

namespace cogip {
namespace parameter_handler {

/// @brief Parameters of a board, read, written and reset through CAN protobuf requests
/// @details Answers only for the parameters of its registry, so that each board on the CAN bus
///          answers for its own. For a Storage::Flash parameter, load() restores the stored
///          value, a set request stores the new value and a reset request erases it.
class ParameterHandler
{
  public:
    /// @brief Construct handler with registry
    /// @param registry Parameters of the board
    explicit ParameterHandler(ParameterRegistry registry) : registry_(registry) {}

    /// @brief Load the parameters stored in flash
    /// @note A parameter whose stored value is rejected keeps its default value.
    void load();

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

    ParameterRegistry registry_; ///< Parameters of the board
};

} // namespace parameter_handler
} // namespace cogip
