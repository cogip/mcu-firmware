// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file motion_control_parameters.cpp
/// @brief Motion control parameters registry and handlers implementation.

// Project includes
#include "motion_control_parameters.hpp"
#include "app_conf.hpp"
#include "parameter/ParameterKeyHash.hpp"
#include "parameter_handler/ParameterHandler.hpp"
#include "platform.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

using namespace cogip::parameter;

/// @brief Parameter key hashes for identification
///
/// @note Each parameter is identified by a unique 32-bit hash computed from its string key.

/// Maximum number of parameters in the registry
constexpr size_t MAX_PARAMETERS_NUMBER = 32;

/// Odometry parameters keys
constexpr uint32_t QDEC_LEFT_POLARITY_KEY = "qdec_left_polarity"_key_hash;
constexpr uint32_t QDEC_RIGHT_POLARITY_KEY = "qdec_right_polarity"_key_hash;
constexpr uint32_t LEFT_WHEEL_DIAMETER_KEY = "left_wheel_diameter_mm"_key_hash;
constexpr uint32_t RIGHT_WHEEL_DIAMETER_KEY = "right_wheel_diameter_mm"_key_hash;
constexpr uint32_t ENCODER_WHEELS_DISTANCE_KEY = "encoder_wheels_distance_mm"_key_hash;
constexpr uint32_t ENCODER_WHEELS_RESOLUTION_KEY = "encoder_wheels_resolution_pulses"_key_hash;

// Parameter handler type
using ParameterHandlerType = parameter_handler::ParameterHandler<MAX_PARAMETERS_NUMBER>;

/// @brief Registry mapping parameter key hashes to parameters object references
///
/// @warning The registry should only contains parameters available for read/write through canpb
static const ParameterHandlerType::Registry registry = {
    /// Odometry parameters
    {QDEC_LEFT_POLARITY_KEY, qdec_left_polarity},
    {QDEC_RIGHT_POLARITY_KEY, qdec_right_polarity},
    {LEFT_WHEEL_DIAMETER_KEY, left_encoder_wheels_diameter_mm},
    {RIGHT_WHEEL_DIAMETER_KEY, right_encoder_wheels_diameter_mm},
    {ENCODER_WHEELS_DISTANCE_KEY, encoder_wheels_distance_mm},
    {ENCODER_WHEELS_RESOLUTION_KEY, encoder_wheels_resolution_pulses},
};

static ParameterHandlerType parameter_handler(registry);

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    PB_ParameterGetResponse response = parameter_handler.handle_get(buffer);
    pf_get_canpb().send_message(parameter_get_response_uuid, &response);
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    PB_ParameterSetResponse response = parameter_handler.handle_set(buffer);
    pf_get_canpb().send_message(parameter_set_response_uuid, &response);
}

} // namespace motion_control
} // namespace pf
} // namespace cogip
