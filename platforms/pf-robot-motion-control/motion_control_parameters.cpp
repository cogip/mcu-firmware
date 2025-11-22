// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file motion_control_parameters.cpp
/// @brief Motion control parameters registry and handlers implementation.

// RIOT includes
#include "log.h"

// Project includes
#include "app_conf.hpp"
#include "canpb/CanProtobuf.hpp"
#include "motion_control_parameters.hpp"
#include "platform.hpp"

// Protobuf messages
#include "PB_ParameterCommands.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

using namespace cogip::parameter;

/// @brief Parameter key hashes for identification
///
/// @note Each parameter is identified by a unique 32-bit hash computed from its string key.

/// Maximum number of parameters in the registry
constexpr uint32_t MAX_PARAMETERS_NUMBER = 32;

/// Odometry parameters keys
constexpr uint32_t QDEC_LEFT_POLARITY_KEY = "qdec_left_polarity"_key_hash;
constexpr uint32_t QDEC_RIGHT_POLARITY_KEY = "qdec_right_polarity"_key_hash;
constexpr uint32_t LEFT_WHEEL_DIAMETER_KEY = "left_wheel_diameter_mm"_key_hash;
constexpr uint32_t RIGHT_WHEEL_DIAMETER_KEY = "right_wheel_diameter_mm"_key_hash;
constexpr uint32_t ENCODER_WHEELS_DISTANCE_KEY = "encoder_wheels_distance_mm"_key_hash;
constexpr uint32_t ENCODER_WHEELS_RESOLUTION_KEY = "encoder_wheels_resolution_pulses"_key_hash;

// Parameter registry type
using ParameterRegistry = etl::map<uint32_t, ParameterBase&, MAX_PARAMETERS_NUMBER>;

/// @brief Registry mapping parameter key hashes to parameters object references
///
/// @warning The registry should only contains parameters available for read/write through canpb
static const ParameterRegistry registry = {
    /// Odometry parameters
    {QDEC_LEFT_POLARITY_KEY, qdec_left_polarity},
    {QDEC_RIGHT_POLARITY_KEY, qdec_right_polarity},
    {LEFT_WHEEL_DIAMETER_KEY, left_encoder_wheels_diameter_mm},
    {RIGHT_WHEEL_DIAMETER_KEY, right_encoder_wheels_diameter_mm},
    {ENCODER_WHEELS_DISTANCE_KEY, encoder_wheels_distance_mm},
    {ENCODER_WHEELS_RESOLUTION_KEY, encoder_wheels_resolution_pulses},
};

/// Static buffers for parameter get/set protobuf messages, reused across CAN requests
static PB_ParameterGetRequest get_request;
static PB_ParameterGetResponse get_response;

static PB_ParameterSetRequest set_request;
static PB_ParameterSetResponse set_response;

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    // Clear previous messages
    get_request.clear();
    get_response.clear();

    // Deserialize request
    EmbeddedProto::Error error = get_request.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Parameter get: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    // Prepare response by re-use incoming key has
    get_response.set_key_hash(get_request.get_key_hash());

    // Check if parameter exist in the registry
    auto it = registry.find(get_request.get_key_hash());
    if (it != registry.end()) {
        // Convert parameter value to protobuf message
        if (!it->second.pb_copy(get_response.mutable_value())) {
            LOG_ERROR("Fail to copy parameter value\n");
            return;
        }
    } else {
        LOG_ERROR("Parameter not found!\n");
    }

    // Send response
    pf_get_canpb().send_message(parameter_get_response_uuid, &get_response);
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    // Clear previous messages
    set_request.clear();
    set_response.clear();

    // Deserialize request
    EmbeddedProto::Error error = set_request.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Parameter set: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    // Prepare response by re-use incoming key has
    set_response.set_key_hash(set_request.get_key_hash());
    auto it = registry.find(set_request.get_key_hash());
    if (it != registry.end()) {
        // Convert parameter value from protobuf message
        if (!it->second.canWrite()) {
            LOG_WARNING("- key_hash: 0x%08" PRIx32 ", read only parameter\n", it->first);
            set_response.set_status(PB_ParameterStatus::READ_ONLY);
        } else if (it->second.pb_read(set_request.mutable_value())) {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n", it->first);
            set_response.set_status(PB_ParameterStatus::SUCCESS);
        } else {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter update failed\n", it->first);
            set_response.set_status(PB_ParameterStatus::VALIDATION_FAILED);
        }
    } else {
        LOG_INFO("Parameter not found!\n");
        set_response.set_status(PB_ParameterStatus::NOT_FOUND);
    }

    // Send response
    pf_get_canpb().send_message(parameter_set_response_uuid, &set_response);
}

} // namespace motion_control
} // namespace pf
} // namespace cogip
