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
using parameter_handler::ParameterBoundValue;
using parameter_handler::ParameterDescriptor;
using parameter_handler::param_tag_bits;

namespace {

/// @brief Build a float ParameterBoundValue
constexpr ParameterBoundValue float_bound(float value)
{
    ParameterBoundValue bound{};
    bound.type = PB_ParameterType::PARAM_TYPE_FLOAT;
    bound.float_value = value;
    return bound;
}

/// @brief Descriptor factory for an unbounded float parameter
ParameterDescriptor float_param(
    uint32_t key_hash,
    const char* name,
    uint32_t tags_bitmask,
    bool read_only,
    parameter::ParameterBase& param)
{
    return ParameterDescriptor{
        key_hash,
        name,
        PB_ParameterType::PARAM_TYPE_FLOAT,
        tags_bitmask,
        read_only,
        /*has_bounds=*/false,
        ParameterBoundValue{},
        ParameterBoundValue{},
        &param,
    };
}

/// @brief Descriptor factory for a float parameter with a closed bound range
ParameterDescriptor float_param_bounded(
    uint32_t key_hash,
    const char* name,
    uint32_t tags_bitmask,
    bool read_only,
    parameter::ParameterBase& param,
    float min_value,
    float max_value)
{
    return ParameterDescriptor{
        key_hash,
        name,
        PB_ParameterType::PARAM_TYPE_FLOAT,
        tags_bitmask,
        read_only,
        /*has_bounds=*/true,
        float_bound(min_value),
        float_bound(max_value),
        &param,
    };
}

} // namespace

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
    float_param(QDEC_LEFT_POLARITY_KEY, "qdec_left_polarity",
                tag_polarity_encoder, /*read_only=*/true, qdec_left_polarity),
    float_param(QDEC_RIGHT_POLARITY_KEY, "qdec_right_polarity",
                tag_polarity_encoder, /*read_only=*/true, qdec_right_polarity),
    float_param(LEFT_WHEEL_DIAMETER_KEY, "left_wheel_diameter_mm",
                tag_wheel_geometry, /*read_only=*/false, left_encoder_wheels_diameter_mm),
    float_param(RIGHT_WHEEL_DIAMETER_KEY, "right_wheel_diameter_mm",
                tag_wheel_geometry, /*read_only=*/false, right_encoder_wheels_diameter_mm),
    float_param(ENCODER_WHEELS_DISTANCE_KEY, "encoder_wheels_distance_mm",
                tag_wheel_geometry, /*read_only=*/false, encoder_wheels_distance_mm),
    float_param(ENCODER_WHEELS_RESOLUTION_KEY, "encoder_wheels_resolution_pulses",
                tag_encoder_resolution, /*read_only=*/true, encoder_wheels_resolution_pulses),

    /// QuadPID chain PID parameters
    // Linear pose PID
    float_param(LINEAR_POSE_PID_KP_KEY, "linear_pose_pid_kp",
                tag_quadpid_linear_pose, /*read_only=*/false, linear_pose_pid_kp),
    float_param(LINEAR_POSE_PID_KI_KEY, "linear_pose_pid_ki",
                tag_quadpid_linear_pose, /*read_only=*/false, linear_pose_pid_ki),
    float_param(LINEAR_POSE_PID_KD_KEY, "linear_pose_pid_kd",
                tag_quadpid_linear_pose, /*read_only=*/false, linear_pose_pid_kd),
    // Angular pose PID
    float_param(ANGULAR_POSE_PID_KP_KEY, "angular_pose_pid_kp",
                tag_quadpid_angular_pose, /*read_only=*/false, angular_pose_pid_kp),
    float_param(ANGULAR_POSE_PID_KI_KEY, "angular_pose_pid_ki",
                tag_quadpid_angular_pose, /*read_only=*/false, angular_pose_pid_ki),
    float_param(ANGULAR_POSE_PID_KD_KEY, "angular_pose_pid_kd",
                tag_quadpid_angular_pose, /*read_only=*/false, angular_pose_pid_kd),
    // Linear speed PID
    float_param(LINEAR_SPEED_PID_KP_KEY, "linear_speed_pid_kp",
                tag_quadpid_linear_speed, /*read_only=*/false, linear_speed_pid_kp),
    float_param(LINEAR_SPEED_PID_KI_KEY, "linear_speed_pid_ki",
                tag_quadpid_linear_speed, /*read_only=*/false, linear_speed_pid_ki),
    float_param(LINEAR_SPEED_PID_KD_KEY, "linear_speed_pid_kd",
                tag_quadpid_linear_speed, /*read_only=*/false, linear_speed_pid_kd),
    // Angular speed PID
    float_param(ANGULAR_SPEED_PID_KP_KEY, "angular_speed_pid_kp",
                tag_quadpid_angular_speed, /*read_only=*/false, angular_speed_pid_kp),
    float_param(ANGULAR_SPEED_PID_KI_KEY, "angular_speed_pid_ki",
                tag_quadpid_angular_speed, /*read_only=*/false, angular_speed_pid_ki),
    float_param(ANGULAR_SPEED_PID_KD_KEY, "angular_speed_pid_kd",
                tag_quadpid_angular_speed, /*read_only=*/false, angular_speed_pid_kd),

    /// Tracker chain PID parameters
    // Tracker linear pose PID
    float_param(TRACKER_LINEAR_POSE_PID_KP_KEY, "tracker_linear_pose_pid_kp",
                tag_tracker_linear_pose, /*read_only=*/false, tracker_linear_pose_pid_kp),
    float_param(TRACKER_LINEAR_POSE_PID_KI_KEY, "tracker_linear_pose_pid_ki",
                tag_tracker_linear_pose, /*read_only=*/false, tracker_linear_pose_pid_ki),
    float_param(TRACKER_LINEAR_POSE_PID_KD_KEY, "tracker_linear_pose_pid_kd",
                tag_tracker_linear_pose, /*read_only=*/false, tracker_linear_pose_pid_kd),
    // Tracker angular pose PID
    float_param(TRACKER_ANGULAR_POSE_PID_KP_KEY, "tracker_angular_pose_pid_kp",
                tag_tracker_angular_pose, /*read_only=*/false, tracker_angular_pose_pid_kp),
    float_param(TRACKER_ANGULAR_POSE_PID_KI_KEY, "tracker_angular_pose_pid_ki",
                tag_tracker_angular_pose, /*read_only=*/false, tracker_angular_pose_pid_ki),
    float_param(TRACKER_ANGULAR_POSE_PID_KD_KEY, "tracker_angular_pose_pid_kd",
                tag_tracker_angular_pose, /*read_only=*/false, tracker_angular_pose_pid_kd),
    // Tracker linear speed PID
    float_param(TRACKER_LINEAR_SPEED_PID_KP_KEY, "tracker_linear_speed_pid_kp",
                tag_tracker_linear_speed, /*read_only=*/false, tracker_linear_speed_pid_kp),
    float_param(TRACKER_LINEAR_SPEED_PID_KI_KEY, "tracker_linear_speed_pid_ki",
                tag_tracker_linear_speed, /*read_only=*/false, tracker_linear_speed_pid_ki),
    float_param(TRACKER_LINEAR_SPEED_PID_KD_KEY, "tracker_linear_speed_pid_kd",
                tag_tracker_linear_speed, /*read_only=*/false, tracker_linear_speed_pid_kd),
    // Tracker angular speed PID
    float_param(TRACKER_ANGULAR_SPEED_PID_KP_KEY, "tracker_angular_speed_pid_kp",
                tag_tracker_angular_speed, /*read_only=*/false, tracker_angular_speed_pid_kp),
    float_param(TRACKER_ANGULAR_SPEED_PID_KI_KEY, "tracker_angular_speed_pid_ki",
                tag_tracker_angular_speed, /*read_only=*/false, tracker_angular_speed_pid_ki),
    float_param(TRACKER_ANGULAR_SPEED_PID_KD_KEY, "tracker_angular_speed_pid_kd",
                tag_tracker_angular_speed, /*read_only=*/false, tracker_angular_speed_pid_kd),

    /// Pose straight filter thresholds (Clamp policy bounds)
    float_param_bounded(LINEAR_THRESHOLD_KEY, "linear_threshold",
                        tag_pose_threshold_linear, /*read_only=*/false,
                        param_linear_threshold, 1.0f, 10.0f),
    float_param_bounded(ANGULAR_THRESHOLD_KEY, "angular_threshold",
                        tag_pose_threshold_angular, /*read_only=*/false,
                        param_angular_threshold, 1.0f, 5.0f),
    float_param_bounded(ANGULAR_INTERMEDIATE_THRESHOLD_KEY, "angular_intermediate_threshold",
                        tag_pose_threshold_angular, /*read_only=*/false,
                        param_angular_intermediate_threshold, 1.0f, 5.0f),

    /// Speed and acceleration limits
    float_param(MIN_SPEED_LINEAR_KEY, "min_speed_linear",
                tag_speed_limit_linear, /*read_only=*/false, param_min_speed_linear),
    float_param(MAX_SPEED_LINEAR_KEY, "max_speed_linear",
                tag_speed_limit_linear, /*read_only=*/false, param_max_speed_linear),
    float_param(MAX_ACC_LINEAR_KEY, "max_acc_linear",
                tag_accel_limit_linear, /*read_only=*/false, param_max_acc_linear),
    float_param(MAX_DEC_LINEAR_KEY, "max_dec_linear",
                tag_accel_limit_linear, /*read_only=*/false, param_max_dec_linear),
    float_param(MIN_SPEED_ANGULAR_KEY, "min_speed_angular",
                tag_speed_limit_angular, /*read_only=*/false, param_min_speed_angular),
    float_param(MAX_SPEED_ANGULAR_KEY, "max_speed_angular",
                tag_speed_limit_angular, /*read_only=*/false, param_max_speed_angular),
    float_param(MAX_ACC_ANGULAR_KEY, "max_acc_angular",
                tag_accel_limit_angular, /*read_only=*/false, param_max_acc_angular),
    float_param(MAX_DEC_ANGULAR_KEY, "max_dec_angular",
                tag_accel_limit_angular, /*read_only=*/false, param_max_dec_angular),

#ifdef ROBOT_HAS_OTOS
    /// OTOS localization calibration scalars
    float_param(OTOS_LINEAR_SCALAR_KEY, "otos_linear_scalar",
                tag_otos_linear, /*read_only=*/false, otos_linear_scalar),
    float_param(OTOS_ANGULAR_SCALAR_KEY, "otos_angular_scalar",
                tag_otos_angular, /*read_only=*/false, otos_angular_scalar),
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

} // namespace motion_control
} // namespace pf
} // namespace cogip
