//// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
//// This file is subject to the terms and conditions of the GNU Lesser
//// General Public License v2.1. See the file LICENSE in the top level
//// directory for more details.

/// @file motion_control_parameters.hpp
/// @brief Motion control parameter definitions and registry
///
/// @note This file defines all parameter for motion control configuration.

#include "etl/map.h"

#include "localization/LocalizationDifferentialParameters.hpp"
#include "parameter/Parameter.hpp"
#include "parameter/ParameterKeyHash.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

using namespace cogip::parameter;
using namespace cogip::localization;

/// @brief Parameter key hashes for identification
///
/// @note Each parameter is identified by a unique 32-bit hash computed from its string key.

/// Maximum number of parameters in the registry
constexpr uint32_t MAX_PARAMETERS_NUMBER = 32;

/// Odometry properties
constexpr uint32_t LEFT_WHEEL_DIAMETER_KEY = "left_wheel_diameter_mm"_key_hash;
constexpr uint32_t RIGHT_WHEEL_DIAMETER_KEY = "right_wheel_diameter_mm"_key_hash;
constexpr uint32_t WHEELS_DISTANCE_KEY = "wheels_distance_mm"_key_hash;

/// @brief Handle parameter get request from canpb
///
/// @note Processes a parameter get request received via CAN bus, retrieves the requested
///       parameter value from the registry, and sends back a response.
///
/// @param[in] buffer CAN protocol buffer containing the serialized parameter get request
void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer);

/// @brief Handle parameter set request from CAN bus
///
/// @note Processes a parameter set request received via CAN bus, updates the parameter
///       value in the registry, and sends back a response with the operation status.
///
/// @param[in] buffer CAN protocol buffer containing the serialized parameter set request
void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer);

const LocalizationDifferentialParameters& pf_localization_parameters();

} // namespace motion_control
} // namespace pf
} // namespace cogip
