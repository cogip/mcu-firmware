// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_power_monitor
/// @{
/// @file
/// @brief       Power monitor parameters structures definition
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstdint>

// RIOT includes
#include <periph/gpio.h>

#include "max1161x.h"

namespace cogip {
namespace power_monitor {

/// @brief Measurement chain of a power rail
/// @details The rail voltage reaches the ADC through a voltage divider, and its current through
///          a shunt resistor followed by a current sense amplifier.
struct RailParameters
{
    /// ADC channel connected to the rail measurement chain.
    max1161x_channel_t channel;

    /// Voltage divider top resistor in ohm.
    uint32_t divider_top_ohm = 0;

    /// Voltage divider bottom resistor in ohm.
    uint32_t divider_bottom_ohm = 0;

    /// Current shunt resistor in milliohm.
    uint32_t shunt_mohm = 0;

    /// Current sense amplifier gain in V/V.
    uint32_t current_sense_gain = 0;
};

/// @brief Power monitor parameters
struct PowerMonitorParameters
{
    /// MAX1161X ADC device parameters, configured for unipolar conversions.
    max1161x_params_t adc_params;

    /// ADC reference voltage in millivolts.
    uint32_t adc_vref_mv = 0;

    /// Measurement mode selection pin: high to measure voltages, low to measure currents.
    gpio_t mode_pin;

    /// Analog switch settling time after a measurement mode change in milliseconds.
    uint32_t mode_settling_time_ms = 0;
};

} // namespace power_monitor
} // namespace cogip

/// @}
