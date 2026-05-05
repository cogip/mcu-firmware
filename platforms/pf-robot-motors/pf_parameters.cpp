// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file pf_parameters.cpp
/// @brief Lift motor PID parameters registry and handlers implementation.

// Set to 1 to dump every registered parameter and its post-load value at boot.
#define ENABLE_DEBUG 0

#include <inttypes.h>

// RIOT includes
#include "debug.h"
#include "log.h"

// Project includes
#include "app_conf.hpp"
#include "parameter_handler/ParameterHandler.hpp"
#include "pf_common/platform_common.hpp"
#include "pf_common/uuids.hpp"
#include "pf_parameters.hpp"

namespace cogip {
namespace pf {
namespace parameters {

using namespace cogip::parameter;

/// Maximum number of parameters in the registry
constexpr size_t MAX_PARAMETERS_NUMBER = 16;

// Parameter handler type
using ParameterHandlerType = parameter_handler::ParameterHandler<MAX_PARAMETERS_NUMBER>;

/// @brief Registry mapping parameter key hashes to parameters object references
///
/// @warning The registry should only contain parameters available for read/write through canpb
static const ParameterHandlerType::Registry registry = {
    /// Lift PID parameters
    // Pose PID
    {cogip::app::actuators::LIFT_POSE_PID_KP_KEY, cogip::app::actuators::motor_lift_pose_pid_kp},
    {cogip::app::actuators::LIFT_POSE_PID_KI_KEY, cogip::app::actuators::motor_lift_pose_pid_ki},
    {cogip::app::actuators::LIFT_POSE_PID_KD_KEY, cogip::app::actuators::motor_lift_pose_pid_kd},
    // Speed PID
    {cogip::app::actuators::LIFT_SPEED_PID_KP_KEY, cogip::app::actuators::motor_lift_speed_pid_kp},
    {cogip::app::actuators::LIFT_SPEED_PID_KI_KEY, cogip::app::actuators::motor_lift_speed_pid_ki},
    {cogip::app::actuators::LIFT_SPEED_PID_KD_KEY, cogip::app::actuators::motor_lift_speed_pid_kd},
    // Brake speed PID
    {cogip::app::actuators::LIFT_BRAKE_SPEED_PID_KP_KEY,
     cogip::app::actuators::motor_lift_brake_speed_pid_kp},
    {cogip::app::actuators::LIFT_BRAKE_SPEED_PID_KI_KEY,
     cogip::app::actuators::motor_lift_brake_speed_pid_ki},
    {cogip::app::actuators::LIFT_BRAKE_SPEED_PID_KD_KEY,
     cogip::app::actuators::motor_lift_brake_speed_pid_kd},
};

static ParameterHandlerType parameter_handler(registry);

void pf_load_parameters()
{
    for (auto& entry : registry) {
#if ENABLE_DEBUG
        // Snapshot the default before load(): at construction value_ == default_value_.
        PB_ParameterValue pb_default;
        entry.second.pb_copy(pb_default);
#endif
        if (!entry.second.load()) {
            LOG_WARNING("Parameter 0x%08" PRIx32 ": flash load failed, using default\n",
                        entry.first);
        }
#if ENABLE_DEBUG
        PB_ParameterValue pb_value;
        if (!entry.second.pb_copy(pb_value)) {
            DEBUG("Parameter 0x%08" PRIx32 ": pb_copy failed\n", entry.first);
            continue;
        }
        using FN = PB_ParameterValue::FieldNumber;
        switch (pb_value.get_which_value()) {
        case FN::FLOAT_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %f (default=%f) (float)\n", entry.first,
                  static_cast<double>(pb_value.float_value()),
                  static_cast<double>(pb_default.float_value()));
            break;
        case FN::DOUBLE_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %f (default=%f) (double)\n", entry.first,
                  pb_value.double_value(), pb_default.double_value());
            break;
        case FN::INT32_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %" PRId32 " (default=%" PRId32 ") (int32)\n",
                  entry.first, pb_value.int32_value(), pb_default.int32_value());
            break;
        case FN::UINT32_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %" PRIu32 " (default=%" PRIu32 ") (uint32)\n",
                  entry.first, pb_value.uint32_value(), pb_default.uint32_value());
            break;
        case FN::INT64_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %" PRId64 " (default=%" PRId64 ") (int64)\n",
                  entry.first, pb_value.int64_value(), pb_default.int64_value());
            break;
        case FN::UINT64_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %" PRIu64 " (default=%" PRIu64 ") (uint64)\n",
                  entry.first, pb_value.uint64_value(), pb_default.uint64_value());
            break;
        case FN::BOOL_VALUE:
            DEBUG("Parameter 0x%08" PRIx32 " = %s (default=%s) (bool)\n", entry.first,
                  pb_value.bool_value() ? "true" : "false",
                  pb_default.bool_value() ? "true" : "false");
            break;
        case FN::NOT_SET:
        default:
            DEBUG("Parameter 0x%08" PRIx32 " = <unset>\n", entry.first);
            break;
        }
#endif
    }
    LOG_INFO("All lift parameters loaded (%u entries)\n", static_cast<unsigned>(registry.size()));
}

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_get(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(pf_common::parameter_get_response_uuid, &response.value());
    }
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_set(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(pf_common::parameter_set_response_uuid, &response.value());
    }
}

void pf_handle_parameter_reset(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_reset(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(pf_common::parameter_reset_response_uuid, &response.value());
    }
}

void init()
{
    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();
    canpb.register_message_handler(pf_common::parameter_get_uuid,
                                   canpb::message_handler_t::create<pf_handle_parameter_get>());
    canpb.register_message_handler(pf_common::parameter_set_uuid,
                                   canpb::message_handler_t::create<pf_handle_parameter_set>());
    canpb.register_message_handler(pf_common::parameter_reset_uuid,
                                   canpb::message_handler_t::create<pf_handle_parameter_reset>());
}

} // namespace parameters
} // namespace pf
} // namespace cogip
