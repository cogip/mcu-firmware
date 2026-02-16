// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterHandler.hpp
/// @brief Generic parameter handler for CAN protobuf get/set requests

#pragma once

// Standard includes
#include <cinttypes>

// RIOT includes
#include "log.h"

// ETL includes
#include "etl/map.h"
#include "etl/optional.h"

// Project includes
#include "canpb/ReadBuffer.hpp"
#include "parameter/ParameterInterface.hpp"

// Protobuf messages
#include "PB_ParameterCommands.hpp"

namespace cogip {
namespace parameter_handler {

/// @brief Generic parameter handler for CAN protobuf get/set requests
/// @tparam MaxParams Maximum number of parameters in the registry
template <size_t MaxParams> class ParameterHandler
{
  public:
    using Registry = etl::map<uint32_t, parameter::ParameterBase&, MaxParams>;

    /// @brief Construct handler with registry reference
    /// @param registry Reference to the parameter registry
    explicit ParameterHandler(const Registry& registry) : registry_(registry) {}

    /// @brief Handle parameter get request
    /// @param buffer CAN read buffer containing serialized PB_ParameterGetRequest
    /// @return Optional response message with parameter value, empty if parameter not found locally
    /// @note Returns empty optional if parameter not in this board's registry, allowing other
    ///       boards on the CAN bus to respond instead. Only the board that owns the parameter
    ///       should send a response.
    etl::optional<PB_ParameterGetResponse> handle_get(canpb::ReadBuffer& buffer)
    {
        PB_ParameterGetRequest request;

        // Deserialize request
        EmbeddedProto::Error error = request.deserialize(buffer);
        if (error != EmbeddedProto::Error::NO_ERRORS) {
            LOG_ERROR("Parameter get: Protobuf deserialization error: %d\n",
                      static_cast<int>(error));
            return etl::nullopt;
        }

        // Check if parameter exists in the registry
        auto it = registry_.find(request.get_key_hash());
        if (it == registry_.end()) {
            // Parameter not in this board's registry - don't respond
            return etl::nullopt;
        }

        // Parameter found - prepare response
        PB_ParameterGetResponse response;
        response.set_key_hash(request.get_key_hash());

        // Convert parameter value to protobuf message
        if (!it->second.pb_copy(response.mutable_value())) {
            LOG_ERROR("Fail to copy parameter value\n");
        }

        return response;
    }

    /// @brief Handle parameter set request
    /// @param buffer CAN read buffer containing serialized PB_ParameterSetRequest
    /// @return Optional response message with operation status, empty if parameter not found
    /// locally
    /// @note Returns empty optional if parameter not in this board's registry, allowing other
    ///       boards on the CAN bus to respond instead. Only the board that owns the parameter
    ///       should send a response.
    etl::optional<PB_ParameterSetResponse> handle_set(canpb::ReadBuffer& buffer)
    {
        PB_ParameterSetRequest request;

        // Deserialize request
        EmbeddedProto::Error error = request.deserialize(buffer);
        if (error != EmbeddedProto::Error::NO_ERRORS) {
            LOG_ERROR("Parameter set: Protobuf deserialization error: %d\n",
                      static_cast<int>(error));
            return etl::nullopt;
        }

        // Check if parameter exists in the registry
        auto it = registry_.find(request.get_key_hash());
        if (it == registry_.end()) {
            // Parameter not in this board's registry - don't respond
            return etl::nullopt;
        }

        // Parameter found - prepare response
        PB_ParameterSetResponse response;
        response.set_key_hash(request.get_key_hash());

        // Try to set parameter value - pb_read() will fail if policy rejects (e.g., ReadOnly)
        if (it->second.pb_read(request.mutable_value())) {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n", it->first);
            response.set_status(PB_ParameterStatus::SUCCESS);
        } else {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter update failed\n", it->first);
            response.set_status(PB_ParameterStatus::VALIDATION_FAILED);
        }

        return response;
    }

  private:
    const Registry& registry_;
};

} // namespace parameter_handler
} // namespace cogip
