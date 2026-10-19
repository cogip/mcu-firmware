// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterHandler.cpp
/// @brief Generic parameter handler implementation

#include "log.h"

// Project includes
#include "parameter_handler/ParameterHandler.hpp"

namespace cogip {
namespace parameter_handler {

void ParameterHandler::load()
{
    for (const auto& entry : registry_) {
        if (!entry.load()) {
            LOG_WARNING("Parameter %s: flash load failed, using default\n", entry.name);
        }
    }
    LOG_INFO("All parameters loaded (%u entries)\n", static_cast<unsigned>(registry_.size()));
}

etl::optional<PB_ParameterGetResponse> ParameterHandler::handle_get(canpb::ReadBuffer& buffer)
{
    PB_ParameterGetRequest request;

    // Deserialize request
    EmbeddedProto::Error error = request.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Parameter get: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return etl::nullopt;
    }

    // Check if parameter exists in the registry
    const ParameterEntry* entry = find(request.get_key_hash());
    if (entry == nullptr) {
        // Parameter not in this board's registry - don't respond
        return etl::nullopt;
    }

    // Parameter found - prepare response
    PB_ParameterGetResponse response;
    response.set_key_hash(request.get_key_hash());

    // Convert parameter value to protobuf message
    if (!entry->parameter.pb_copy(response.mutable_value())) {
        LOG_ERROR("Fail to copy parameter value\n");
    }

    return response;
}

etl::optional<PB_ParameterSetResponse> ParameterHandler::handle_set(canpb::ReadBuffer& buffer)
{
    PB_ParameterSetRequest request;

    // Deserialize request
    EmbeddedProto::Error error = request.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Parameter set: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return etl::nullopt;
    }

    // Check if parameter exists in the registry
    const ParameterEntry* entry = find(request.get_key_hash());
    if (entry == nullptr) {
        // Parameter not in this board's registry - don't respond
        return etl::nullopt;
    }

    // Parameter found - prepare response
    PB_ParameterSetResponse response;
    response.set_key_hash(request.get_key_hash());

    // Try to set parameter value - pb_read() will fail if policy rejects (e.g., ReadOnly)
    if (entry->parameter.pb_read(request.mutable_value())) {
        entry->store();
        LOG_INFO("- %s: parameter updated successfully\n", entry->name);
        response.set_status(PB_ParameterStatus::SUCCESS);
    } else {
        LOG_INFO("- %s: parameter update failed\n", entry->name);
        response.set_status(PB_ParameterStatus::VALIDATION_FAILED);
    }

    return response;
}

etl::optional<PB_ParameterResetResponse> ParameterHandler::handle_reset(canpb::ReadBuffer& buffer)
{
    PB_ParameterResetRequest request;

    // Deserialize request
    EmbeddedProto::Error error = request.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Parameter reset: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return etl::nullopt;
    }

    // Check if parameter exists in the registry
    const ParameterEntry* entry = find(request.get_key_hash());
    if (entry == nullptr) {
        // Parameter not in this board's registry - don't respond
        return etl::nullopt;
    }

    // Parameter found - erase persisted value and restore compile-time default
    entry->erase();
    entry->parameter.reset();

    PB_ParameterResetResponse response;
    response.set_key_hash(request.get_key_hash());
    response.set_status(PB_ParameterStatus::SUCCESS);
    LOG_INFO("- %s: parameter reset to default\n", entry->name);

    return response;
}

const ParameterEntry* ParameterHandler::find(uint32_t key_hash) const
{
    for (const auto& entry : registry_) {
        if (entry.key_hash == key_hash) {
            return &entry;
        }
    }
    return nullptr;
}

} // namespace parameter_handler
} // namespace cogip
