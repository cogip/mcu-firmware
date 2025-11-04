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
#include "localization/LocalizationDifferentialParameters.hpp"
#include "motion_control_parameters.hpp"
#include "platform.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

using namespace cogip::parameter;
using namespace cogip::localization;

// Parameter registry type
using ParameterRegistry = etl::map<uint32_t, ParameterBase&, MAX_PARAMETERS_NUMBER>;

/// Odometry properties
static Parameter<float> left_encoder_wheels_diameter_param{left_encoder_wheels_diameter_mm};
static Parameter<float> right_encoder_wheels_diameter_param{right_encoder_wheels_diameter_mm};
static Parameter<float> encoder_wheels_distance_param{encoder_wheels_distance_mm};
static Parameter<float> left_encoder_polarity{QDEC_LEFT_POLARITY};
static Parameter<float> right_encoder_polarity{QDEC_RIGHT_POLARITY};

/// @brief Registry mapping parameter key hashes to their default values
///
/// @warning The registry should only contains parameters available for modification through canpb
static const ParameterRegistry registry = {
    /// Odometry properties
    {LEFT_WHEEL_DIAMETER_KEY, left_encoder_wheels_diameter_param},
    {RIGHT_WHEEL_DIAMETER_KEY, right_encoder_wheels_diameter_param},
    {WHEELS_DISTANCE_KEY, encoder_wheels_distance_param},
};

static ParameterGetRequest get_request;
static ParameterGetResponse get_response;

static ParameterSetRequest set_request;
static ParameterSetResponse set_response;

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
        if (it->second.pb_copy(get_response.mutable_value())) {
            LOG_ERROR("TODO");
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
        LOG_ERROR("Parameter get: Protobuf deserialization error: %d\n", static_cast<int>(error));
        return;
    }

    // Prepare response by re-use incoming key has
    set_response.set_key_hash(set_request.get_key_hash());

    auto it = registry.find(set_request.get_key_hash());
    if (it != registry.end()) {
        // Convert parameter value from protobuf message
        if (it->second.pb_read(set_request.value())) {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n", it->first);
            set_response.set_status(ParameterStatus::SUCCESS);
        } else {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter update failed\n", it->first);
            set_response.set_status(ParameterStatus::VALIDATION_FAILED);
        }
    } else {
        LOG_INFO("Parameter not found!\n");
        set_response.set_status(ParameterStatus::NOT_FOUND);
    }

    // Send response
    pf_get_canpb().send_message(parameter_set_response_uuid, &set_response);
}

static LocalizationDifferentialParameters localization_params(left_encoder_wheels_diameter_param,
                                                              right_encoder_wheels_diameter_param,
                                                              encoder_wheels_distance_param,
                                                              left_encoder_polarity,
                                                              right_encoder_polarity);

const LocalizationDifferentialParameters& pf_localization_parameters()
{
    return localization_params;
}

} // namespace motion_control
} // namespace pf
} // namespace cogip
