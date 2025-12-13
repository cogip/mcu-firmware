// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_telemetry
/// @{
/// @brief      Generic telemetry sending interface
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

#include "etl/type_traits.h"
#include "mutex.h"
#include "ztimer.h"

#include "KeyHash.hpp"
#include "PB_Telemetry.hpp"
#include "canpb/CanProtobuf.hpp"

namespace cogip {

namespace telemetry {

// Re-export hash utilities from utils for convenience
using cogip::utils::hash_key;
using cogip::utils::operator"" _key_hash;

/// @brief Static class for telemetry data sending
class Telemetry
{
  public:
    /// @brief Initialize telemetry with CAN protocol buffer instance and data UUID
    /// @param canpb CAN protocol buffer instance
    /// @param data_uuid UUID for telemetry data messages
    static void init(cogip::canpb::CanProtobuf& canpb, canpb::uuid_t data_uuid);

    /// @brief Enable telemetry globally
    static void enable();

    /// @brief Disable telemetry globally
    static void disable();

    /// @brief Check if telemetry is enabled
    /// @return true if enabled, false otherwise
    static bool is_enabled();

    /// @brief Send a telemetry data point (if telemetry is enabled)
    /// @tparam T Value type (float, double, int32_t, uint32_t, int64_t, uint64_t)
    /// @param key_hash Hash of the telemetry key (use "my_key"_key_hash)
    /// @param value Value to send
    /// @return true if sent, false if telemetry is disabled or not initialized
    template <typename T> static bool send(uint32_t key_hash, T value)
    {
        static_assert(etl::is_arithmetic<T>::value,
                      "Telemetry::send() only supports arithmetic types");

        mutex_lock(&mutex_);

        // cppcheck-suppress knownConditionTrueFalse
        if (!initialized_ || !enabled_) {
            mutex_unlock(&mutex_);
            return false;
        }

        PB_TelemetryData message;
        message.set_key_hash(key_hash);
        message.set_timestamp_ms(ztimer_now(ZTIMER_MSEC));

        if constexpr (etl::is_same<T, float>::value) {
            message.set_float_value(value);
        } else if constexpr (etl::is_same<T, int32_t>::value) {
            message.set_int32_value(value);
        } else if constexpr (etl::is_same<T, uint32_t>::value) {
            message.set_uint32_value(value);
        } else if constexpr (etl::is_same<T, int64_t>::value) {
            message.set_int64_value(value);
        } else if constexpr (etl::is_same<T, uint64_t>::value) {
            message.set_uint64_value(value);
        }

        canpb_->send_message(uuid_, &message);

        mutex_unlock(&mutex_);
        return true;
    }

  private:
    Telemetry() = delete; // Prevent instantiation

    inline static bool initialized_ = false;                   ///< Initialization flag
    inline static bool enabled_ = false;                       ///< Global telemetry enable flag
    inline static mutex_t mutex_ = MUTEX_INIT;                 ///< Mutex for thread-safe access
    inline static cogip::canpb::CanProtobuf* canpb_ = nullptr; ///< CAN protocol buffer instance
    inline static canpb::uuid_t uuid_ = 0;                     ///< Telemetry message UUID
};

} // namespace telemetry

} // namespace cogip

/// @}
