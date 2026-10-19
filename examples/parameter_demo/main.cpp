// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file main.cpp
/// @brief Simple example demonstrating the parameter library features
///
/// @note Parameters are only accessed through the parameter handler, with canpb get, set and
///       reset requests, as they would be from the CAN bus.

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "etl/optional.h"
#include "log.h"

#include "KeyHash.hpp"
#include "PB_ParameterCommands.hpp"
#include "canpb/ReadBuffer.hpp"
#include "canpb/WriteBuffer.hpp"
#include "flash_kv_storage/FlashKVStorage.hpp"
#include "parameter/Parameter.hpp"
#include "parameter_handler/ParameterHandler.hpp"

using namespace cogip::parameter;
using cogip::parameter_handler::ParameterEntry;
using cogip::parameter_handler::ParameterHandler;
using cogip::utils::operator"" _key_hash;

/// @brief Accepted range of max_angular_velocity, in deg/s
static constexpr ParameterRange<float> max_angular_velocity_range_deg_s{0.0f, 500.0f};

/// @brief Global parameters configuration
static Parameter<float> left_encoder_wheels_diameter_mm{47.8};
static Parameter<float> right_encoder_wheels_diameter_mm{47.8};
static Parameter<float> encoder_wheels_distance_mm{275};
static Parameter<float> encoder_wheels_resolution_pulses{4096 * 4};
static Parameter<float> max_angular_velocity_deg_s{180.0f, max_angular_velocity_range_deg_s};
static Parameter<bool> enable_logging{true};

/// @brief Parameter registry, each parameter identified by its name
///
/// @note This registry demonstrates type-erased heterogeneous parameter storage.
///       Persistent parameters are stored in flash under the hash of their name.
static const ParameterEntry parameter_registry[] = {
    {"left_wheel_diameter", left_encoder_wheels_diameter_mm},
    {"right_wheel_diameter", right_encoder_wheels_diameter_mm},
    {"wheels_distance", encoder_wheels_distance_mm},
    {"wheels_resolution", encoder_wheels_resolution_pulses},
    {"max_angular_velocity", max_angular_velocity_deg_s, ParameterEntry::Store},
    {"enable_logging", enable_logging}};

/// @brief Handler of canpb requests on the registry
static ParameterHandler parameter_handler(parameter_registry);

/// @brief Read buffer of the requests, as received from the CAN bus
static cogip::canpb::ReadBuffer buffer;

/// @brief Serialize a request into the read buffer, as if received from the CAN bus
/// @tparam Message Protobuf request type
/// @param message Request to serialize
template <typename Message> static void to_read_buffer(const Message& message)
{
    cogip::canpb::WriteBuffer write_buffer;
    message.serialize(write_buffer);

    buffer.clear();
    uint8_t* data = write_buffer.get_data();
    for (uint32_t i = 0; i < write_buffer.get_size(); i++) {
        buffer.push(data[i]);
    }
}

/// @brief Hash a parameter name, as the sender of a request does
/// @param name Parameter name
/// @return Key hash of the parameter
static uint32_t key_hash(const char* name)
{
    return cogip::utils::fnv1a_hash_bytes(name, strlen(name));
}

/// @brief Status of a set or reset response
/// @tparam Response Protobuf response type
/// @param response Response returned by the handler
/// @return Status as a string
template <typename Response> static const char* status(const etl::optional<Response>& response)
{
    if (!response.has_value()) {
        return "NOT FOUND";
    }
    return (response->get_status() == PB_ParameterStatus::SUCCESS) ? "SUCCESS" : "FAILED";
}

/// @brief Send a get request through the handler and log the value
/// @param name Parameter name
static void get(const char* name)
{
    PB_ParameterGetRequest request;
    request.set_key_hash(key_hash(name));
    to_read_buffer(request);

    auto response = parameter_handler.handle_get(buffer);
    if (!response.has_value()) {
        LOG_INFO("- Get %s: NOT FOUND\n", name);
        return;
    }

    const auto& value = response->value();
    if (value.has_float_value()) {
        LOG_INFO("- Get %s: %.2f\n", name, value.float_value());
    } else if (value.has_bool_value()) {
        LOG_INFO("- Get %s: %s\n", name, value.bool_value() ? "true" : "false");
    }
}

/// @brief Send a float set request through the handler
/// @param name Parameter name
/// @param value Value to set
static void set(const char* name, float value)
{
    PB_ParameterSetRequest request;
    request.set_key_hash(key_hash(name));
    request.mutable_value().set_float_value(value);
    to_read_buffer(request);

    LOG_INFO("- Set %s to %.1f: %s\n", name, value, status(parameter_handler.handle_set(buffer)));
}

/// @brief Send a reset request through the handler
/// @param name Parameter name
static void reset(const char* name)
{
    PB_ParameterResetRequest request;
    request.set_key_hash(key_hash(name));
    to_read_buffer(request);

    LOG_INFO("- Reset %s: %s\n", name, status(parameter_handler.handle_reset(buffer)));
}

/// @brief Log the value of max_angular_velocity in flash and in memory
static void log_max_angular_velocity()
{
    float stored = 0.0f;
    if (cogip::flash_kv_storage::FlashKVStorage::instance().load("max_angular_velocity"_key_hash,
                                                                 stored) == 0) {
        LOG_INFO("  flash: %.1f\n", stored);
    } else {
        LOG_INFO("  flash: no data\n");
    }
    get("max_angular_velocity");
}

/// @brief Main function demonstrating the parameter library usage patterns
/// @return Always returns 0
int main(void)
{
    // Initialize persistent storage (must happen before load())
    LOG_INFO("=== Load ===\n");
    get("max_angular_velocity");
    if (cogip::flash_kv_storage::FlashKVStorage::instance().init() == 0) {
        // Reload persistent parameters from flash
        parameter_handler.load();
        LOG_INFO("- FlashKVStorage initialized, persistent parameters loaded\n");
        get("max_angular_velocity");
    } else {
        LOG_INFO("- FlashKVStorage initialization failed\n");
    }
    LOG_INFO("\n");

    // clang-format off
    LOG_INFO("=== Compile-time Key Hashing ===\n");
    for (const auto& entry : parameter_registry) {
        LOG_INFO("- '%s'_key_hash: 0x%08" PRIx32 "%s\n", entry.name, entry.key_hash,
                 (entry.key_hash == "max_angular_velocity"_key_hash) ? " (persistent)" : "");
    }
    LOG_INFO("\n");

    LOG_INFO("=== Memory Footprint ===\n");
    LOG_INFO("- float:                                %zu bytes\n", sizeof(float));
    LOG_INFO("- Parameter<float>:                     %zu bytes\n", sizeof(Parameter<float>));
    LOG_INFO("- Parameter<bool>:                      %zu bytes\n", sizeof(Parameter<bool>));
    LOG_INFO("- ParameterBase*:                       %zu bytes\n", sizeof(ParameterBase*));
    LOG_INFO("- ParameterEntry:                       %zu bytes\n", sizeof(ParameterEntry));
    LOG_INFO("\n");
    // clang-format on

    LOG_INFO("=== Get ===\n");
    for (const auto& entry : parameter_registry) {
        get(entry.name);
    }
    get("unknown_parameter");
    LOG_INFO("\n");

    LOG_INFO("=== Set ===\n");
    set("left_wheel_diameter", 48.5f);
    get("left_wheel_diameter");
    LOG_INFO("\n");

    LOG_INFO("=== Range ===\n");
    LOG_INFO("- max_angular_velocity range: [%.1f, %.1f]\n", max_angular_velocity_range_deg_s.min(),
             max_angular_velocity_range_deg_s.max());
    set("max_angular_velocity", 360.0f);
    set("max_angular_velocity", 600.0f);
    set("max_angular_velocity", -50.0f);
    get("max_angular_velocity");

    // Default range rejects NaN
    set("wheels_distance", NAN);

    // Values of another type are rejected
    set("enable_logging", 1.0f);
    LOG_INFO("\n");

    LOG_INFO("=== Flash Persistence ===\n");

    // Reset erases the stored value and restores the default
    reset("max_angular_velocity");
    log_max_angular_velocity();

    // Set stores the new value in flash
    set("max_angular_velocity", 250.0f);
    log_max_angular_velocity();

    // Out-of-range value is rejected, flash NOT updated
    set("max_angular_velocity", 600.0f);
    log_max_angular_velocity();

    LOG_INFO("- Reboot to see max_angular_velocity loaded from flash\n");
    LOG_INFO("\n");

    return 0;
}
