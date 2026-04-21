// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file ParameterHandler.hpp
/// @brief Generic parameter handler for CAN protobuf get/set/announce requests

#pragma once

// Standard includes
#include <cinttypes>
#include <cstddef>

// RIOT includes
#include "log.h"

// ETL includes
#include "etl/optional.h"
#include "etl/vector.h"

// Project includes
#include "canpb/ReadBuffer.hpp"
#include "parameter/ParameterInterface.hpp"
#include "parameter_handler/ParameterDescriptor.hpp"

// Protobuf messages
#include "PB_ParameterCommands.hpp"

namespace cogip {
namespace parameter_handler {

/// @brief Generic parameter handler for CAN protobuf get/set/announce requests
/// @tparam MaxParams Maximum number of parameters in the registry
///
/// @note The registry is a flat `etl::vector<ParameterDescriptor, MaxParams>`.
///       Lookups are linear scans on `key_hash`; with MaxParams ≤ 64 this costs
///       a few microseconds and avoids the per-board map churn we had with
///       `etl::map`. The vector is also the iteration order for the announce
///       stream, so host tools receive parameters in declaration order.
template <size_t MaxParams> class ParameterHandler
{
  public:
    using Registry = etl::vector<ParameterDescriptor, MaxParams>;

    /// @brief Construct handler with registry reference
    /// @param registry Reference to the parameter registry
    explicit ParameterHandler(const Registry& registry) : registry_(registry) {}

    /// @brief Find a descriptor by key hash.
    /// @return Pointer to the descriptor, or nullptr if not found.
    const ParameterDescriptor* find(uint32_t key_hash) const
    {
        for (const auto& entry : registry_) {
            if (entry.key_hash == key_hash) {
                return &entry;
            }
        }
        return nullptr;
    }

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
        const ParameterDescriptor* entry = find(request.get_key_hash());
        if (entry == nullptr) {
            // Parameter not in this board's registry - don't respond
            return etl::nullopt;
        }

        // Parameter found - prepare response
        PB_ParameterGetResponse response;
        response.set_key_hash(request.get_key_hash());

        // Convert parameter value to protobuf message
        if (!entry->param->pb_copy(response.mutable_value())) {
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
        const ParameterDescriptor* entry = find(request.get_key_hash());
        if (entry == nullptr) {
            // Parameter not in this board's registry - don't respond
            return etl::nullopt;
        }

        // Parameter found - prepare response
        PB_ParameterSetResponse response;
        response.set_key_hash(request.get_key_hash());

        // Try to set parameter value - pb_read() will fail if policy rejects (e.g., ReadOnly)
        if (entry->param->pb_read(request.mutable_value())) {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n",
                     entry->key_hash);
            response.set_status(PB_ParameterStatus::SUCCESS);
        } else {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter update failed\n", entry->key_hash);
            response.set_status(PB_ParameterStatus::VALIDATION_FAILED);
        }

        return response;
    }

    /// @brief Count descriptors matching a tag filter.
    /// @param filter PB_ParameterTag used as a bit index; PARAM_TAG_NONE matches all.
    /// @return Number of descriptors that pass the filter.
    size_t count_matching(PB_ParameterTag filter) const
    {
        size_t count = 0;
        for (const auto& entry : registry_) {
            if (matches_filter(entry, filter)) {
                ++count;
            }
        }
        return count;
    }

    /// @brief Iterate over descriptors matching a tag filter.
    /// @param filter PB_ParameterTag used as a bit index; PARAM_TAG_NONE matches all.
    /// @param callback Invocable accepting `(const ParameterDescriptor&, size_t index)`.
    ///
    /// @note `index` is the 0-based position within the filtered stream (not within
    ///       the registry), suitable for filling PB_ParameterAnnounceHeader.index.
    template <typename Callback>
    void for_each_matching(PB_ParameterTag filter, Callback&& callback) const
    {
        size_t index = 0;
        for (const auto& entry : registry_) {
            if (matches_filter(entry, filter)) {
                callback(entry, index);
                ++index;
            }
        }
    }

  private:
    /// @brief Test whether a descriptor's tags include the requested filter.
    static bool matches_filter(const ParameterDescriptor& entry, PB_ParameterTag filter)
    {
        if (filter == PB_ParameterTag::PARAM_TAG_NONE) {
            return true;
        }
        const uint32_t mask = 1u << static_cast<uint32_t>(filter);
        return (entry.tags_bitmask & mask) != 0;
    }

    const Registry& registry_;
};

} // namespace parameter_handler
} // namespace cogip
