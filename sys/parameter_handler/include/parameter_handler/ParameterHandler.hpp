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
    /// @return Response message with parameter value
    PB_ParameterGetResponse handle_get(canpb::ReadBuffer& buffer)
    {
        PB_ParameterGetRequest request;
        PB_ParameterGetResponse response;

        // Deserialize request
        EmbeddedProto::Error error = request.deserialize(buffer);
        if (error != EmbeddedProto::Error::NO_ERRORS) {
            LOG_ERROR("Parameter get: Protobuf deserialization error: %d\n",
                      static_cast<int>(error));
            return response;
        }

        // Prepare response by re-use incoming key hash
        response.set_key_hash(request.get_key_hash());

        // Check if parameter exists in the registry
        auto it = registry_.find(request.get_key_hash());
        if (it != registry_.end()) {
            // Convert parameter value to protobuf message
            if (!it->second.pb_copy(response.mutable_value())) {
                LOG_ERROR("Fail to copy parameter value\n");
            }
        } else {
            LOG_ERROR("Parameter not found!\n");
        }

        return response;
    }

    /// @brief Handle parameter set request
    /// @param buffer CAN read buffer containing serialized PB_ParameterSetRequest
    /// @return Response message with operation status
    PB_ParameterSetResponse handle_set(canpb::ReadBuffer& buffer)
    {
        PB_ParameterSetRequest request;
        PB_ParameterSetResponse response;

        // Deserialize request
        EmbeddedProto::Error error = request.deserialize(buffer);
        if (error != EmbeddedProto::Error::NO_ERRORS) {
            LOG_ERROR("Parameter set: Protobuf deserialization error: %d\n",
                      static_cast<int>(error));
            return response;
        }

        // Prepare response by re-use incoming key hash
        response.set_key_hash(request.get_key_hash());
        auto it = registry_.find(request.get_key_hash());
        if (it != registry_.end()) {
            // Try to set parameter value - pb_read() will fail if policy rejects (e.g., ReadOnly)
            if (it->second.pb_read(request.mutable_value())) {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n",
                         it->first);
                response.set_status(PB_ParameterStatus::SUCCESS);
            } else {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter update failed\n", it->first);
                response.set_status(PB_ParameterStatus::VALIDATION_FAILED);
            }
        } else {
            LOG_INFO("Parameter not found!\n");
            response.set_status(PB_ParameterStatus::NOT_FOUND);
        }

        return response;
    }

  private:
    const Registry& registry_;
};

} // namespace parameter_handler
} // namespace cogip
