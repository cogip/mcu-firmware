//// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
//// This file is subject to the terms and conditions of the GNU Lesser
//// General Public License v2.1. See the file LICENSE in the top level
//// directory for more details.

/// @file motion_control_parameters.hpp
/// @brief Motion control parameter definitions and registry
///
/// @note This file defines all parameter for motion control configuration.

#pragma once

#include "canpb/ReadBuffer.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

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

} // namespace motion_control
} // namespace pf
} // namespace cogip
