// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file main.cpp
/// @brief Simple example demonstrating the parameter library features

#include <cinttypes>
#include <cstdio>

#include "log.h"

#include "PB_ParameterCommands.hpp"
#include "etl/array.h"
#include "etl/map.h"
#include "etl/string.h"
#include "parameter/Parameter.hpp"
#include "parameter/ParameterKeyHash.hpp"

using namespace cogip::parameter;

/// @brief Parameter key hashes for identification
///
/// @note Each parameter is identified by a unique 32-bit hash computed from its string key.
constexpr uint32_t KEY_LEFT_WHEEL_DIAMETER = "left_wheel_diameter"_key_hash;
constexpr uint32_t KEY_RIGHT_WHEEL_DIAMETER = "right_wheel_diameter"_key_hash;
constexpr uint32_t KEY_WHEELS_DISTANCE = "wheels_distance"_key_hash;
constexpr uint32_t KEY_WHEELS_RESOLUTION = "wheels_resolution"_key_hash;
constexpr uint32_t KEY_MAX_ANGULAR_VELOCITY = "max_angular_velocity"_key_hash;
constexpr uint32_t KEY_ENABLE_LOGGING = "enable_logging"_key_hash;

// clang-format off

/// @brief Global parameters configuration
static Parameter<float> left_encoder_wheels_diameter_mm{47.8};
static Parameter<float> right_encoder_wheels_diameter_mm{47.8};
static Parameter<float> encoder_wheels_distance_mm{275};
static Parameter<float> encoder_wheels_resolution_pulses{4096 * 4};
static Parameter<float, WithBounds<0, 500>> max_angular_velocity_deg_s{180.0}; // With validation: 0-500°/s
static Parameter<bool> enable_logging{true};

/// @brief Parameter registry mapping key hashes to parameter references
///
/// @note This registry demonstrates type-erased heterogeneous parameter storage.
constexpr size_t MAX_PARAMETERS = 10;
static etl::map<uint32_t, ParameterBase&, MAX_PARAMETERS> parameter_registry = {
    {KEY_LEFT_WHEEL_DIAMETER, left_encoder_wheels_diameter_mm},   // float
    {KEY_RIGHT_WHEEL_DIAMETER, right_encoder_wheels_diameter_mm}, // float
    {KEY_WHEELS_DISTANCE, encoder_wheels_distance_mm},            // float
    {KEY_WHEELS_RESOLUTION, encoder_wheels_resolution_pulses},    // float
    {KEY_MAX_ANGULAR_VELOCITY, max_angular_velocity_deg_s},       // float with bounds [0, 500]
    {KEY_ENABLE_LOGGING, enable_logging}                          // bool
};

// clang-format on

/// @brief Aggregates localization parameters for read-only access
///
/// @note This structure groups related parameters using const references to ParameterInterface.
/// The references are intentionally const to prevent modification by user classes.
///
/// @note The underlying parameters can still be modified through their global instances,
///       but not through this aggregation structure.
struct localizationParameters
{
    /// @brief Constructs the parameter aggregation structure
    /// @param left_encoder_wheels_diameter_mm Left encoder wheel diameter parameter
    /// @param right_encoder_wheels_diameter_mm Right encoder wheel diameter parameter
    /// @param encoder_wheels_distance_mm Distance between encoder wheels parameter
    /// @param encoder_wheels_resolution_pulses Encoder wheels resolution parameter
    localizationParameters(const ParameterInterface<float>& left_encoder_wheels_diameter_mm,
                           const ParameterInterface<float>& right_encoder_wheels_diameter_mm,
                           const ParameterInterface<float>& encoder_wheels_distance_mm,
                           const ParameterInterface<float>& encoder_wheels_resolution_pulses)
        : left_encoder_wheels_diameter_mm(left_encoder_wheels_diameter_mm),
          right_encoder_wheels_diameter_mm(right_encoder_wheels_diameter_mm),
          encoder_wheels_distance_mm(encoder_wheels_distance_mm),
          encoder_wheels_resolution_pulses(encoder_wheels_resolution_pulses)
    {
    }

    /// Read only-parameter. Each parameter have it's own getter.
    const ParameterInterface<float>& left_encoder_wheels_diameter_mm;
    const ParameterInterface<float>& right_encoder_wheels_diameter_mm;
    const ParameterInterface<float>& encoder_wheels_distance_mm;
    const ParameterInterface<float>& encoder_wheels_resolution_pulses;
};

localizationParameters parameters(left_encoder_wheels_diameter_mm, right_encoder_wheels_diameter_mm,
                                  encoder_wheels_distance_mm, encoder_wheels_resolution_pulses);

/// @brief Main function demonstrating the parameter library usage patterns
/// @return Always returns 0
int main(void)
{
    // clang-format off
    LOG_INFO("=== Compile-time Key Hashing ===\n");
    LOG_INFO("- 'left_wheel_diameter'_key_hash:  0x%08" PRIx32 "\n", KEY_LEFT_WHEEL_DIAMETER);
    LOG_INFO("- 'right_wheel_diameter'_key_hash: 0x%08" PRIx32 "\n", KEY_RIGHT_WHEEL_DIAMETER);
    LOG_INFO("- 'wheels_distance'_key_hash:      0x%08" PRIx32 "\n", KEY_WHEELS_DISTANCE);
    LOG_INFO("- 'wheels_resolution'_key_hash:    0x%08" PRIx32 "\n", KEY_WHEELS_RESOLUTION);
    LOG_INFO("- 'max_angular_velocity'_key_hash: 0x%08" PRIx32 "\n", KEY_MAX_ANGULAR_VELOCITY);
    LOG_INFO("- 'enable_logging'_key_hash:       0x%08" PRIx32 "\n", KEY_ENABLE_LOGGING);
    LOG_INFO("\n");

    LOG_INFO("=== Memory Footprint ===\n");
    LOG_INFO("- float:                                %zu bytes\n", sizeof(float));
    LOG_INFO("- Parameter<float>:                     %zu bytes\n", sizeof(Parameter<float>));
    LOG_INFO("- Parameter<float, WithBounds<0, 500>>: %zu bytes\n", sizeof(Parameter<float, WithBounds<0, 500>>));
    LOG_INFO("- Parameter<bool>:                      %zu bytes\n", sizeof(Parameter<bool>));
    LOG_INFO("- ParameterBase*:                       %zu bytes\n", sizeof(ParameterBase*));
    LOG_INFO("\n");

    LOG_INFO("=== Validation Policy ===\n");
    LOG_INFO("- max_angular_velocity initial: %.1f deg/s (valid: %s)\n",
             max_angular_velocity_deg_s.get(),
             max_angular_velocity_deg_s.isValid() ? "yes" : "no");

    // Try to set valid value within bounds [0, 500]
    if (max_angular_velocity_deg_s.set(360.0f)) {
        LOG_INFO("- Set to 360.0 deg/s: SUCCESS (within bounds [0, 500])\n");
    } else {
        LOG_INFO("- Set to 360.0 deg/s: FAILED\n");
    }

    // Try to set invalid value (exceeds upper bound)
    if (max_angular_velocity_deg_s.set(600.0f)) {
        LOG_INFO("- Set to 600.0 deg/s: SUCCESS\n");
    } else {
        LOG_INFO("- Set to 600.0 deg/s: FAILED (exceeds upper bound 500)\n");
    }

    // Try to set invalid value (below lower bound)
    if (max_angular_velocity_deg_s.set(-50.0f)) {
        LOG_INFO("- Set to -50.0 deg/s: SUCCESS\n");
    } else {
        LOG_INFO("- Set to -50.0 deg/s: FAILED (below lower bound 0)\n");
    }

    LOG_INFO("- max_angular_velocity current: %.1f deg/s\n", max_angular_velocity_deg_s.get());
    LOG_INFO("\n");

    LOG_INFO("=== Initial values (Direct access) ===\n");
    LOG_INFO("- left_encoder_wheels_diameter_mm: %.2f mm\n", left_encoder_wheels_diameter_mm.get());
    LOG_INFO("- right_encoder_wheels_diameter_mm: %.2f mm\n", right_encoder_wheels_diameter_mm.get());
    LOG_INFO("- encoder_wheels_distance_mm: %.2f mm\n", encoder_wheels_distance_mm.get());
    LOG_INFO("- encoder_wheels_resolution_pulses: %.0f\n", encoder_wheels_resolution_pulses.get());
    LOG_INFO("\n");

    LOG_INFO("=== Initial values (Polymorphic access) ===\n");
    LOG_INFO("- parameters.left_encoder_wheels_diameter_mm: %.2f mm\n", parameters.left_encoder_wheels_diameter_mm.get());
    LOG_INFO("- parameters.right_encoder_wheels_diameter_mm: %.2f mm\n", parameters.right_encoder_wheels_diameter_mm.get());
    LOG_INFO("- parameters.encoder_wheels_distance_mm: %.2f mm\n", parameters.encoder_wheels_distance_mm.get());
    LOG_INFO("- parameters.encoder_wheels_resolution_pulses: %.0f\n", parameters.encoder_wheels_resolution_pulses.get());
    LOG_INFO("\n");

    // clang-format on

    // Protobuf conversion demonstration - Generic approach using registry
    // Shows how to update parameters from ParameterSetRequest messages (from CAN)
    LOG_INFO("=== Protobuf Conversion (ParameterSetRequest -> Parameter) ===\n");

    PB_ParameterSetRequest set_request;
    PB_ParameterSetResponse set_response;

    // Simulate receiving ParameterSetRequest messages
    set_request.set_key_hash(KEY_LEFT_WHEEL_DIAMETER);
    set_request.mutable_value().set_float_value(48.5f);

    set_response.set_key_hash(set_request.get_key_hash());

    auto it = parameter_registry.find(set_request.get_key_hash());
    if (it != parameter_registry.end()) {
        // Add parameter to response message.
        // No value means no parameter available for this hash.
        if (it->second.pb_read(set_request.value())) {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n", it->first);
            set_response.set_status(PB_ParameterStatus::SUCCESS);
        } else {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", deserialization failed\n", it->first);
            set_response.set_status(PB_ParameterStatus::VALIDATION_FAILED);
        }
    } else {
        LOG_INFO("Parameter not found!\n");
        set_response.set_status(PB_ParameterStatus::NOT_FOUND);
    }
    LOG_INFO("\n");

    // Verify via polymorphic access
    LOG_INFO("=== Verification (polymorphic access) ===\n");
    LOG_INFO("- parameters.left_encoder_wheels_diameter_mm: %.2f mm\n",
             parameters.left_encoder_wheels_diameter_mm.get());
    LOG_INFO("\n");

    // Protobuf conversion demonstration - Generic approach using registry
    // Shows how to build ParameterGetResponse messages (for CAN communication)
    LOG_INFO("=== Protobuf Conversion (Parameter -> ParameterGetResponse) ===\n");

    PB_ParameterGetRequest get_request;
    PB_ParameterGetResponse get_response;

    // Simulate request through canpb
    get_request.set_key_hash(KEY_LEFT_WHEEL_DIAMETER);

    // Prepare response through canpb.
    get_response.set_key_hash(get_request.get_key_hash());

    it = parameter_registry.find(get_request.get_key_hash());
    if (it != parameter_registry.end()) {
        // Add parameter to response message.
        // No value means no parameter available for this hash.
        if (it->second.pb_copy(get_response.mutable_value())) {
            // Display value for testing purpose
            // clang-format off
            const auto& value = get_response.value();
            if (value.has_float_value()) {
                LOG_INFO("- Found: key_hash: 0x%08" PRIx32 ", float value: %.2f\n", it->first, value.float_value());
            } else if (value.has_double_value()) {
                LOG_INFO("- Found: key_hash: 0x%08" PRIx32 ", double value: %.2f\n", it->first, value.double_value());
            } else if (value.has_int32_value()) {
                LOG_INFO("- Found: key_hash: 0x%08" PRIx32 ", int32 value: %" PRId32 "\n", it->first, value.int32_value());
            } else if (value.has_bool_value()) {
                LOG_INFO("- Found: key_hash: 0x%08" PRIx32 ", bool value: %s\n", it->first, value.bool_value() ? "true" : "false");
            }
            // clang-format on
        }
    } else {
        LOG_INFO("Parameter not found!\n");
    }
    LOG_INFO("\n");

    return 0;
}
