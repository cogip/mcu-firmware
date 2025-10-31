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
/// Each parameter is identified by a unique 32-bit hash computed from its string key.
///
/// @details Design rationale:
/// - **Compile-time hashing**: The _key_hash user-defined literal computes FNV-1a hashes
///   at compile-time using constexpr. This means zero runtime overhead for hash computation.
///
/// - **Consistent algorithm**: Uses the same FNV-1a algorithm as etl::fnv_1a_32, ensuring
///   compatibility between compile-time and runtime hash computations.
///
/// - **Compact identifiers**: 32-bit hashes are smaller than strings, reducing memory usage
///   and improving lookup performance in the registry map.
///
/// - **CAN bus compatibility**: The hash serves as the parameter identifier in protobuf
///   messages (ParameterGetRequest/Response, ParameterSetRequest/Response), enabling
///   efficient parameter addressing over CAN.
///
/// - **Type-safe constants**: Using constexpr uint32_t ensures the hashes are computed once
///   at compile-time and can be used in constant expressions throughout the code.
constexpr uint32_t KEY_LEFT_WHEEL_DIAMETER = "left_wheel_diameter"_key_hash;
constexpr uint32_t KEY_RIGHT_WHEEL_DIAMETER = "right_wheel_diameter"_key_hash;
constexpr uint32_t KEY_WHEELS_DISTANCE = "wheels_distance"_key_hash;
constexpr uint32_t KEY_WHEELS_RESOLUTION = "wheels_resolution"_key_hash;
constexpr uint32_t KEY_MAX_ANGULAR_VELOCITY = "max_angular_velocity"_key_hash;
constexpr uint32_t KEY_ENABLE_LOGGING = "enable_logging"_key_hash;

/// Global parameters configuration (different types!)
///
/// @details Design rationale:
/// - **Static storage**: Parameters are declared static to ensure their lifetime extends
///   throughout the program. This is necessary because the registry stores references.
///
/// - **Heterogeneous types**: Parameters can have different types (float, int32_t, bool).
///   The type-erased registry (ParameterBase&) allows storing all these in one container.
///
/// - **Default serialization**: All parameters support protobuf serialization by default
///   through ParameterBase's virtual interface.
///
/// - **Type-specific access**: While the registry provides type-erased access, parameters
///   can still be accessed directly with full type information for type-safe operations.
///
/// - **Validation policies**: Parameters can have validation bounds (e.g., WithBounds<0, 500>)
///   to ensure values remain within acceptable ranges. Invalid values are rejected during set().
static Parameter<float> left_encoder_wheels_diameter_mm{47.8};
static Parameter<float> right_encoder_wheels_diameter_mm{47.8};
static Parameter<float> encoder_wheels_distance_mm{275};
static Parameter<float> encoder_wheels_resolution_pulses{4096 * 4};
static Parameter<float, WithBounds<0, 500>> max_angular_velocity_deg_s{180.0}; // With validation: 0-500°/s
static Parameter<bool> enable_logging{true};

/// @brief Parameter registry mapping key hashes to parameter references
///
/// This registry demonstrates type-erased heterogeneous parameter storage.
///
/// @details Design rationale:
/// - **Type erasure via ParameterBase**: The map stores references to ParameterBase,
///   which is the non-template base class. This allows storing parameters of different
///   types (float, int32_t, bool, etc.) in a single homogeneous container.
///
/// - **References instead of pointers**: Using ParameterBase& instead of ParameterBase*
///   provides stronger lifetime guarantees and cleaner syntax (. instead of ->).
///   References cannot be null and must be initialized.
///
/// - **Static storage**: Parameters are declared static to ensure their lifetime matches
///   the registry's lifetime. This is crucial since the registry stores references.
///
/// - **Key-based lookup**: The registry enables generic parameter access by key_hash,
///   which is essential for CAN communication where parameters are identified by their hash.
///
/// - **Serialization without type knowledge**: Through ParameterBase's virtual serialize()
///   and deserialize() methods, we can serialize any parameter without knowing its concrete
///   type at compile-time. The ParameterValue protobuf message handles the type dispatch.
constexpr size_t MAX_PARAMETERS = 10;
static etl::map<uint32_t, ParameterBase&, MAX_PARAMETERS> parameter_registry = {
    {KEY_LEFT_WHEEL_DIAMETER, left_encoder_wheels_diameter_mm},   // float
    {KEY_RIGHT_WHEEL_DIAMETER, right_encoder_wheels_diameter_mm}, // float
    {KEY_WHEELS_DISTANCE, encoder_wheels_distance_mm},            // float
    {KEY_WHEELS_RESOLUTION, encoder_wheels_resolution_pulses},    // float
    {KEY_MAX_ANGULAR_VELOCITY, max_angular_velocity_deg_s},       // float with bounds [0, 500]
    {KEY_ENABLE_LOGGING, enable_logging}                          // bool
};

/// @brief Aggregates localization parameters for read-only access
///
/// This structure groups related parameters using const references to ParameterInterface.
/// The references are intentionally const to prevent modification by user classes.
///
/// @details
/// Design rationale:
/// - Parameters are stored as const references to ParameterInterface<T>
/// - This provides polymorphic access while enforcing read-only semantics
/// - Modifications must be performed through direct parameter access (e.g.,
/// left_encoder_wheels_diameter_mm.set())
/// - User classes receive this structure to read parameters without the ability to modify them
/// - This pattern separates parameter configuration (direct access) from parameter usage (via
/// structure)
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
///
/// This example demonstrates multiple parameter library features:
/// 1. Direct access: Read and write operations on global Parameter instances
/// 2. Polymorphic access: Read-only operations through the localizationParameters structure
/// 3. Compile-time key hashing: Using hash_key() and _key literal
/// 4. Protobuf serialization: Converting parameters to/from ParameterValue messages
///
/// @details
/// The demonstration shows:
/// - Compile-time key hash computation
/// - Memory footprint comparison between native types and Parameter<T>
/// - Initial parameter values accessed both directly and through the aggregation structure
/// - Serialization of parameters to ParameterValue messages
/// - Deserialization from ParameterValue messages back into parameters
///
/// @return Always returns 0
int main(void)
{
    // clang-format off
    // Key hash demonstration
    LOG_INFO("=== Compile-time Key Hashing ===\n");
    LOG_INFO("- 'left_wheel_diameter'_key_hash:  0x%08" PRIx32 "\n", KEY_LEFT_WHEEL_DIAMETER);
    LOG_INFO("- 'right_wheel_diameter'_key_hash: 0x%08" PRIx32 "\n", KEY_RIGHT_WHEEL_DIAMETER);
    LOG_INFO("- 'wheels_distance'_key_hash:      0x%08" PRIx32 "\n", KEY_WHEELS_DISTANCE);
    LOG_INFO("- 'wheels_resolution'_key_hash:    0x%08" PRIx32 "\n", KEY_WHEELS_RESOLUTION);
    LOG_INFO("- 'max_angular_velocity'_key_hash: 0x%08" PRIx32 "\n", KEY_MAX_ANGULAR_VELOCITY);
    LOG_INFO("- 'enable_logging'_key_hash:       0x%08" PRIx32 "\n", KEY_ENABLE_LOGGING);
    LOG_INFO("\n");

    // Memory footprint comparison
    LOG_INFO("=== Memory Footprint Comparison ===\n");
    LOG_INFO("- float:                                %zu bytes\n", sizeof(float));
    LOG_INFO("- Parameter<float>:                     %zu bytes\n", sizeof(Parameter<float>));
    LOG_INFO("- Parameter<float, WithBounds<0, 500>>: %zu bytes\n", sizeof(Parameter<float, WithBounds<0, 500>>));
    LOG_INFO("- Parameter<bool>:                      %zu bytes\n", sizeof(Parameter<bool>));
    LOG_INFO("- ParameterBase*:                       %zu bytes\n", sizeof(ParameterBase*));
    LOG_INFO("\n");

    // Validation demonstration
    LOG_INFO("=== Validation Policy Demonstration ===\n");
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

    // Direct parameter access (read/write)
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

    // Protobuf serialization demonstration - Generic approach using registry
    // Shows how to build ParameterSetRequest messages (for CAN communication)
    //
    // Design rationale:
    // - Type-erased iteration: We iterate over the registry using ParameterBase&,
    //   which works for parameters of any type (float, int32_t, bool, etc.).
    // - Polymorphic serialization: param.serialize() is a virtual method that
    //   dispatches to the correct type-specific implementation at runtime.
    // - Generic CAN communication: This pattern allows building ParameterSetRequest
    //   messages for any parameter without type-specific code in the communication layer.
    LOG_INFO("=== Protobuf Serialization (Parameter -> ParameterSetRequest) ===\n");

    ParameterSetRequest set_request;

    // Iterate over all parameters in the registry (works for any type!)
    for (const auto& entry : parameter_registry) {
        uint32_t key_hash = entry.first;
        ParameterBase& param = entry.second;

        // Build ParameterSetRequest message
        set_request.set_key_hash(key_hash);
        if (param.serialize(set_request.mutable_value())) {
            // Display value (assuming float for this demo, but could be any type)
            const auto& value = set_request.value();
            if (value.has_float_value()) {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", float value: %.2f\n",
                         key_hash, value.float_value());
            } else if (value.has_double_value()) {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", double value: %.2f\n",
                         key_hash, value.double_value());
            } else if (value.has_int32_value()) {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", int32 value: %" PRId32 "\n",
                         key_hash, value.int32_value());
            } else if (value.has_bool_value()) {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", bool value: %s\n",
                         key_hash, value.bool_value() ? "true" : "false");
            }
        }
    }

    LOG_INFO("\n");

    // Deserialization demonstration - Generic approach using registry
    // Shows how to update parameters from ParameterGetResponse messages (from CAN)
    //
    // Design rationale:
    // - Key-based dispatch: Parameters are looked up by their key_hash, allowing
    //   the communication layer to route messages without knowing parameter types.
    // - Polymorphic deserialization: param.deserialize() handles type-specific
    //   deserialization and validation automatically through virtual dispatch.
    // - Error handling: The registry lookup can fail (NOT_FOUND) and deserialization
    //   can fail (validation error), both handled gracefully.
    // - Validation integration: When deserializing, the parameter's validation policy
    //   is automatically applied, ensuring type safety and bounds checking.
    LOG_INFO("=== Protobuf Deserialization (ParameterGetResponse -> Parameter) ===\n");

    ParameterGetResponse get_response;

    // Simulate receiving ParameterGetResponse messages for different parameters
    struct SetCommand {
        uint32_t key_hash;
        float value;
    };

    etl::array<SetCommand, 2> commands = {{
        {KEY_LEFT_WHEEL_DIAMETER, 48.5f},
        {KEY_RIGHT_WHEEL_DIAMETER, 47.2f}
    }};

    for (const auto& cmd : commands) {
        // Find parameter in registry by key_hash
        auto it = parameter_registry.find(cmd.key_hash);
        if (it != parameter_registry.end()) {
            // Build ParameterGetResponse message
            get_response.set_key_hash(cmd.key_hash);
            get_response.mutable_value().set_float_value(cmd.value);

            // Deserialize into parameter
            ParameterBase& param = it->second;
            if (param.deserialize(get_response.value())) {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", parameter updated successfully\n",
                         cmd.key_hash);
            } else {
                LOG_INFO("- key_hash: 0x%08" PRIx32 ", deserialization failed\n", cmd.key_hash);
            }
        } else {
            LOG_INFO("- key_hash: 0x%08" PRIx32 ", NOT_FOUND in registry\n", cmd.key_hash);
        }
    }

    LOG_INFO("\n");

    // Verify via polymorphic access
    LOG_INFO("=== Verification (polymorphic access) ===\n");
    LOG_INFO("- parameters.left_encoder_wheels_diameter_mm: %.2f mm\n",
             parameters.left_encoder_wheels_diameter_mm.get());
    LOG_INFO("- parameters.right_encoder_wheels_diameter_mm: %.2f mm\n",
             parameters.right_encoder_wheels_diameter_mm.get());
    LOG_INFO("\n");
    // clang-format on

    return 0;
}
