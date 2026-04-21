// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file motion_control_parameters.cpp
/// @brief Motion control parameters registry and handlers implementation.

// RIOT includes
#include "log.h"

// Project includes
#include "KeyHash.hpp"
#include "motion_control_parameters.hpp"
#include "parameter_handler/ParameterDescriptor.hpp"
#include "parameter_handler/ParameterHandler.hpp"
#include "platform.hpp"

namespace cogip {
namespace pf {
namespace motion_control {

/// Maximum number of parameters in the registry
constexpr size_t MAX_PARAMETERS_NUMBER = 64;

// Parameter handler type
using ParameterHandlerType = parameter_handler::ParameterHandler<MAX_PARAMETERS_NUMBER>;

/// @brief Board identifier: FNV-1a hash of the platform name.
///
/// @note Included in every PB_ParameterAnnounceHeader so the host can route
///       parameters to their owning board without a hard-coded table.
constexpr uint32_t platform_board_id = "pf-robot-motion-control"_key_hash;

// ----------------------------------------------------------------------------
// Local shortcuts to keep the registry readable
// ----------------------------------------------------------------------------
using parameter_handler::ParameterDescriptor;
using parameter_handler::param_tag_bits;

// ----------------------------------------------------------------------------
// Tag groupings (kept local to this TU to keep the registry entries short)
// ----------------------------------------------------------------------------
namespace {

constexpr uint32_t tag_polarity_encoder =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_POLARITY, PB_ParameterTag::PARAM_TAG_ENCODER);

constexpr uint32_t tag_wheel_geometry =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_WHEEL_GEOMETRY,
                   PB_ParameterTag::PARAM_TAG_ENCODER,
                   PB_ParameterTag::PARAM_TAG_LOCALIZATION);

constexpr uint32_t tag_encoder_resolution =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_ENCODER,
                   PB_ParameterTag::PARAM_TAG_LOCALIZATION);

constexpr uint32_t tag_quadpid_linear_pose =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_QUADPID,
                   PB_ParameterTag::PARAM_TAG_LINEAR,
                   PB_ParameterTag::PARAM_TAG_POSE);

constexpr uint32_t tag_quadpid_angular_pose =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_QUADPID,
                   PB_ParameterTag::PARAM_TAG_ANGULAR,
                   PB_ParameterTag::PARAM_TAG_POSE);

constexpr uint32_t tag_quadpid_linear_speed =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_QUADPID,
                   PB_ParameterTag::PARAM_TAG_LINEAR,
                   PB_ParameterTag::PARAM_TAG_SPEED);

constexpr uint32_t tag_quadpid_angular_speed =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_QUADPID,
                   PB_ParameterTag::PARAM_TAG_ANGULAR,
                   PB_ParameterTag::PARAM_TAG_SPEED);

constexpr uint32_t tag_tracker_linear_pose =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_TRACKER,
                   PB_ParameterTag::PARAM_TAG_LINEAR,
                   PB_ParameterTag::PARAM_TAG_POSE);

constexpr uint32_t tag_tracker_angular_pose =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_TRACKER,
                   PB_ParameterTag::PARAM_TAG_ANGULAR,
                   PB_ParameterTag::PARAM_TAG_POSE);

constexpr uint32_t tag_tracker_linear_speed =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_TRACKER,
                   PB_ParameterTag::PARAM_TAG_LINEAR,
                   PB_ParameterTag::PARAM_TAG_SPEED);

constexpr uint32_t tag_tracker_angular_speed =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_PID,
                   PB_ParameterTag::PARAM_TAG_TRACKER,
                   PB_ParameterTag::PARAM_TAG_ANGULAR,
                   PB_ParameterTag::PARAM_TAG_SPEED);

constexpr uint32_t tag_pose_threshold_linear =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_POSE_THRESHOLD,
                   PB_ParameterTag::PARAM_TAG_LINEAR);

constexpr uint32_t tag_pose_threshold_angular =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_POSE_THRESHOLD,
                   PB_ParameterTag::PARAM_TAG_ANGULAR);

constexpr uint32_t tag_speed_limit_linear =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_SPEED_LIMIT,
                   PB_ParameterTag::PARAM_TAG_LINEAR);

constexpr uint32_t tag_speed_limit_angular =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_SPEED_LIMIT,
                   PB_ParameterTag::PARAM_TAG_ANGULAR);

constexpr uint32_t tag_accel_limit_linear =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_ACCELERATION_LIMIT,
                   PB_ParameterTag::PARAM_TAG_LINEAR);

constexpr uint32_t tag_accel_limit_angular =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_ACCELERATION_LIMIT,
                   PB_ParameterTag::PARAM_TAG_ANGULAR);

#ifdef ROBOT_HAS_OTOS
constexpr uint32_t tag_otos_linear =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_OTOS,
                   PB_ParameterTag::PARAM_TAG_LOCALIZATION,
                   PB_ParameterTag::PARAM_TAG_LINEAR);

constexpr uint32_t tag_otos_angular =
    param_tag_bits(PB_ParameterTag::PARAM_TAG_OTOS,
                   PB_ParameterTag::PARAM_TAG_LOCALIZATION,
                   PB_ParameterTag::PARAM_TAG_ANGULAR);
#endif

} // namespace

// ----------------------------------------------------------------------------
// Registry
// ----------------------------------------------------------------------------
/// @brief Registry mapping parameters to their metadata (type, tags, bounds).
///
/// @warning The registry must only contain parameters available for read/write
///          through canpb. The vector is filled once at static-init time and
///          never mutated.
static const ParameterHandlerType::Registry registry = {
    /// Odometry parameters
    DECLARE_PARAM(QDEC_LEFT_POLARITY_KEY, "qdec_left_polarity",
                  tag_polarity_encoder, qdec_left_polarity),
    DECLARE_PARAM(QDEC_RIGHT_POLARITY_KEY, "qdec_right_polarity",
                  tag_polarity_encoder, qdec_right_polarity),
    DECLARE_PARAM(LEFT_WHEEL_DIAMETER_KEY, "left_wheel_diameter_mm",
                  tag_wheel_geometry, left_encoder_wheels_diameter_mm),
    DECLARE_PARAM(RIGHT_WHEEL_DIAMETER_KEY, "right_wheel_diameter_mm",
                  tag_wheel_geometry, right_encoder_wheels_diameter_mm),
    DECLARE_PARAM(ENCODER_WHEELS_DISTANCE_KEY, "encoder_wheels_distance_mm",
                  tag_wheel_geometry, encoder_wheels_distance_mm),
    DECLARE_PARAM(ENCODER_WHEELS_RESOLUTION_KEY, "encoder_wheels_resolution_pulses",
                  tag_encoder_resolution, encoder_wheels_resolution_pulses),

    /// QuadPID chain PID parameters
    DECLARE_PARAM(LINEAR_POSE_PID_KP_KEY, "linear_pose_pid_kp",
                  tag_quadpid_linear_pose, linear_pose_pid_kp),
    DECLARE_PARAM(LINEAR_POSE_PID_KI_KEY, "linear_pose_pid_ki",
                  tag_quadpid_linear_pose, linear_pose_pid_ki),
    DECLARE_PARAM(LINEAR_POSE_PID_KD_KEY, "linear_pose_pid_kd",
                  tag_quadpid_linear_pose, linear_pose_pid_kd),
    DECLARE_PARAM(ANGULAR_POSE_PID_KP_KEY, "angular_pose_pid_kp",
                  tag_quadpid_angular_pose, angular_pose_pid_kp),
    DECLARE_PARAM(ANGULAR_POSE_PID_KI_KEY, "angular_pose_pid_ki",
                  tag_quadpid_angular_pose, angular_pose_pid_ki),
    DECLARE_PARAM(ANGULAR_POSE_PID_KD_KEY, "angular_pose_pid_kd",
                  tag_quadpid_angular_pose, angular_pose_pid_kd),
    DECLARE_PARAM(LINEAR_SPEED_PID_KP_KEY, "linear_speed_pid_kp",
                  tag_quadpid_linear_speed, linear_speed_pid_kp),
    DECLARE_PARAM(LINEAR_SPEED_PID_KI_KEY, "linear_speed_pid_ki",
                  tag_quadpid_linear_speed, linear_speed_pid_ki),
    DECLARE_PARAM(LINEAR_SPEED_PID_KD_KEY, "linear_speed_pid_kd",
                  tag_quadpid_linear_speed, linear_speed_pid_kd),
    DECLARE_PARAM(ANGULAR_SPEED_PID_KP_KEY, "angular_speed_pid_kp",
                  tag_quadpid_angular_speed, angular_speed_pid_kp),
    DECLARE_PARAM(ANGULAR_SPEED_PID_KI_KEY, "angular_speed_pid_ki",
                  tag_quadpid_angular_speed, angular_speed_pid_ki),
    DECLARE_PARAM(ANGULAR_SPEED_PID_KD_KEY, "angular_speed_pid_kd",
                  tag_quadpid_angular_speed, angular_speed_pid_kd),

    /// Tracker chain PID parameters
    DECLARE_PARAM(TRACKER_LINEAR_POSE_PID_KP_KEY, "tracker_linear_pose_pid_kp",
                  tag_tracker_linear_pose, tracker_linear_pose_pid_kp),
    DECLARE_PARAM(TRACKER_LINEAR_POSE_PID_KI_KEY, "tracker_linear_pose_pid_ki",
                  tag_tracker_linear_pose, tracker_linear_pose_pid_ki),
    DECLARE_PARAM(TRACKER_LINEAR_POSE_PID_KD_KEY, "tracker_linear_pose_pid_kd",
                  tag_tracker_linear_pose, tracker_linear_pose_pid_kd),
    DECLARE_PARAM(TRACKER_ANGULAR_POSE_PID_KP_KEY, "tracker_angular_pose_pid_kp",
                  tag_tracker_angular_pose, tracker_angular_pose_pid_kp),
    DECLARE_PARAM(TRACKER_ANGULAR_POSE_PID_KI_KEY, "tracker_angular_pose_pid_ki",
                  tag_tracker_angular_pose, tracker_angular_pose_pid_ki),
    DECLARE_PARAM(TRACKER_ANGULAR_POSE_PID_KD_KEY, "tracker_angular_pose_pid_kd",
                  tag_tracker_angular_pose, tracker_angular_pose_pid_kd),
    DECLARE_PARAM(TRACKER_LINEAR_SPEED_PID_KP_KEY, "tracker_linear_speed_pid_kp",
                  tag_tracker_linear_speed, tracker_linear_speed_pid_kp),
    DECLARE_PARAM(TRACKER_LINEAR_SPEED_PID_KI_KEY, "tracker_linear_speed_pid_ki",
                  tag_tracker_linear_speed, tracker_linear_speed_pid_ki),
    DECLARE_PARAM(TRACKER_LINEAR_SPEED_PID_KD_KEY, "tracker_linear_speed_pid_kd",
                  tag_tracker_linear_speed, tracker_linear_speed_pid_kd),
    DECLARE_PARAM(TRACKER_ANGULAR_SPEED_PID_KP_KEY, "tracker_angular_speed_pid_kp",
                  tag_tracker_angular_speed, tracker_angular_speed_pid_kp),
    DECLARE_PARAM(TRACKER_ANGULAR_SPEED_PID_KI_KEY, "tracker_angular_speed_pid_ki",
                  tag_tracker_angular_speed, tracker_angular_speed_pid_ki),
    DECLARE_PARAM(TRACKER_ANGULAR_SPEED_PID_KD_KEY, "tracker_angular_speed_pid_kd",
                  tag_tracker_angular_speed, tracker_angular_speed_pid_kd),

    /// Pose straight filter thresholds (bounds auto-derived from Clamp policy)
    DECLARE_PARAM(LINEAR_THRESHOLD_KEY, "linear_threshold",
                  tag_pose_threshold_linear, param_linear_threshold),
    DECLARE_PARAM(ANGULAR_THRESHOLD_KEY, "angular_threshold",
                  tag_pose_threshold_angular, param_angular_threshold),
    DECLARE_PARAM(ANGULAR_INTERMEDIATE_THRESHOLD_KEY, "angular_intermediate_threshold",
                  tag_pose_threshold_angular, param_angular_intermediate_threshold),

    /// Speed and acceleration limits
    DECLARE_PARAM(MIN_SPEED_LINEAR_KEY, "min_speed_linear",
                  tag_speed_limit_linear, param_min_speed_linear),
    DECLARE_PARAM(MAX_SPEED_LINEAR_KEY, "max_speed_linear",
                  tag_speed_limit_linear, param_max_speed_linear),
    DECLARE_PARAM(MAX_ACC_LINEAR_KEY, "max_acc_linear",
                  tag_accel_limit_linear, param_max_acc_linear),
    DECLARE_PARAM(MAX_DEC_LINEAR_KEY, "max_dec_linear",
                  tag_accel_limit_linear, param_max_dec_linear),
    DECLARE_PARAM(MIN_SPEED_ANGULAR_KEY, "min_speed_angular",
                  tag_speed_limit_angular, param_min_speed_angular),
    DECLARE_PARAM(MAX_SPEED_ANGULAR_KEY, "max_speed_angular",
                  tag_speed_limit_angular, param_max_speed_angular),
    DECLARE_PARAM(MAX_ACC_ANGULAR_KEY, "max_acc_angular",
                  tag_accel_limit_angular, param_max_acc_angular),
    DECLARE_PARAM(MAX_DEC_ANGULAR_KEY, "max_dec_angular",
                  tag_accel_limit_angular, param_max_dec_angular),

#ifdef ROBOT_HAS_OTOS
    /// OTOS localization calibration scalars
    DECLARE_PARAM(OTOS_LINEAR_SCALAR_KEY, "otos_linear_scalar",
                  tag_otos_linear, otos_linear_scalar),
    DECLARE_PARAM(OTOS_ANGULAR_SCALAR_KEY, "otos_angular_scalar",
                  tag_otos_angular, otos_angular_scalar),
#endif
};

static ParameterHandlerType parameter_handler(registry);

void pf_load_parameters()
{
    for (const auto& entry : registry) {
        if (!entry.param->load()) {
            LOG_WARNING("Parameter 0x%08" PRIx32 ": flash load failed, using default\n",
                        entry.key_hash);
        }
    }
    LOG_INFO("All parameters loaded (%u entries)\n", static_cast<unsigned>(registry.size()));
}

void pf_handle_parameter_get(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_get(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(parameter_get_response_uuid, &response.value());
    }
}

void pf_handle_parameter_set(cogip::canpb::ReadBuffer& buffer)
{
    auto response = parameter_handler.handle_set(buffer);
    // Only respond if this board owns the parameter
    if (response.has_value()) {
        pf_get_canpb().send_message(parameter_set_response_uuid, &response.value());
    }
}

/// @brief Maximum length of a parameter name on the wire.
///
/// @note 36 chars keeps PB_ParameterAnnounceName under the 48 B binary / 64 B
///       base64 single-frame budget of canpb (key_hash 5 B + string overhead
///       2 B + 36 B payload ≈ 43 B → base64 ≈ 60 B).
constexpr uint32_t PARAMETER_NAME_MAX_LENGTH = 36;

/// @brief Stream every parameter matching `tag_filter` as announce frames.
///
/// @note Factored out of pf_handle_parameter_announce so the
///       copilot-connected hook (and unit tests) can trigger an announce
///       without a canpb request buffer.
static void _announce_parameters(PB_ParameterTag tag_filter)
{
    const size_t total_count = parameter_handler.count_matching(tag_filter);

    // No matching parameter on this board: stay silent so other boards can answer.
    if (total_count == 0) {
        return;
    }

    cogip::canpb::CanProtobuf& canpb = pf_get_canpb();

    parameter_handler.for_each_matching(
        tag_filter,
        [&canpb, total_count](const parameter_handler::ParameterDescriptor& entry, size_t index) {
            uint32_t flags = 0;
            if (entry.read_only) {
                flags |= 0x1u;
            }
            if (entry.has_bounds) {
                flags |= 0x2u;
            }

            // Header frame
            PB_ParameterAnnounceHeader header;
            header.set_board_id(platform_board_id);
            header.set_key_hash(entry.key_hash);
            header.set_type(entry.type);
            header.set_tags_bitmask(entry.tags_bitmask);
            header.set_flags(flags);
            header.set_total_count(static_cast<uint32_t>(total_count));
            header.set_index(static_cast<uint32_t>(index));
            canpb.send_message(parameter_announce_header_uuid, &header);

            // Name frame (separate message because the string does not fit in the
            // header frame budget).
            PB_ParameterAnnounceName<PARAMETER_NAME_MAX_LENGTH> name_msg;
            name_msg.set_key_hash(entry.key_hash);
            name_msg.mutable_name() = entry.name;
            canpb.send_message(parameter_announce_name_uuid, &name_msg);

            // Bounds frame (only when the parameter has a closed range).
            if (entry.has_bounds) {
                PB_ParameterAnnounceBounds bounds_msg;
                bounds_msg.set_key_hash(entry.key_hash);
                entry.min_value.pb_copy(bounds_msg.mutable_min_value());
                entry.max_value.pb_copy(bounds_msg.mutable_max_value());
                canpb.send_message(parameter_announce_bounds_uuid, &bounds_msg);
            }
        });
}

void pf_handle_parameter_announce(cogip::canpb::ReadBuffer& buffer)
{
    PB_ParameterAnnounceRequest request;
    EmbeddedProto::Error error = request.deserialize(buffer);
    if (error != EmbeddedProto::Error::NO_ERRORS) {
        LOG_ERROR("Parameter announce: Protobuf deserialization error: %d\n",
                  static_cast<int>(error));
        return;
    }

    _announce_parameters(request.get_tag_filter());
}

void pf_auto_announce_parameters()
{
    _announce_parameters(PB_ParameterTag::PARAM_TAG_NONE);
}

} // namespace motion_control
} // namespace pf
} // namespace cogip
