// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    lib_power_monitor Power monitor library
/// @ingroup     lib
/// @brief       Power rails voltage and current monitoring
/// @{
/// @file
/// @brief       Power rails voltage and current monitor
/// @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>

#pragma once

#include <cstddef>
#include <cstdint>

// RIOT includes
#include "max1161x.h"
#include "mutex.h"

// ETL includes
#include "etl/array.h"
#include "etl/span.h"

// Project includes
#include "gpio/gpio.hpp"
#include "power_monitor/PowerMonitorParameters.hpp"
#include "power_monitor/RailMeasure.hpp"

namespace cogip {
namespace power_monitor {

/// @brief Power rails voltage and current monitor
/// @details Each ADC channel is shared between the voltage divider and the current sense
///          amplifier of a rail. An analog switch, driven by the mode pin, selects which one
///          reaches the ADC, so all voltages are sampled first, then all currents.
class PowerMonitor
{
  public:
    /// @brief Maximum number of monitored rails, one per ADC channel
    static constexpr size_t RAILS_MAX = MAX1161X_NUM_CHANNELS;

    /// @brief Constructor, initializes the mode pin
    /// @tparam N Number of monitored rails
    /// @param[in] parameters Power monitor parameters, must outlive the monitor
    /// @param[in] rails      Monitored rails, must outlive the monitor
    template <size_t N>
    PowerMonitor(const PowerMonitorParameters& parameters, const RailParameters (&rails)[N])
        : parameters_(parameters), rails_(rails), mode_gpio_(parameters.mode_pin, GPIO_OUT)
    {
        static_assert(N <= RAILS_MAX, "More rails than ADC channels");
    }

    /// @brief Check the rails parameters, initialize the ADC and select the voltage measurement
    ///        mode
    /// @return 0 on success, -EINVAL if a rail has a null divider bottom resistor, shunt resistor
    ///         or current sense gain, other negative errno code otherwise
    int init();

    /// @brief Sample the voltage then the current of every rail
    /// @details Blocks for twice the mode settling time plus the ADC conversions.
    ///          A failing voltage sampling does not prevent the current sampling, and
    ///          conversely: values not measured keep their previous measure.
    /// @return 0 on success, first negative errno code encountered otherwise
    int update();

    /// @brief Get the number of monitored rails
    /// @return Number of monitored rails
    size_t rails_count() const
    {
        return rails_.size();
    }

    /// @brief Get the last measure of a rail
    /// @param[in] index Rail index, in the order of the rails given to the constructor
    /// @return Last measure of the rail
    RailMeasure measure(size_t index) const;

  private:
    /// @brief Measurement mode of the analog switch
    enum class Mode : uint8_t {
        VOLTAGE = 0, ///< Voltage dividers outputs routed to the ADC
        CURRENT = 1, ///< Current sense amplifiers outputs routed to the ADC
    };

    /// @brief Select the measurement mode and wait for the analog switch to settle
    /// @param[in] mode Measurement mode to select
    void select_mode(Mode mode);

    /// @brief Select the voltage mode and measure the voltage of every rail
    /// @details A failing conversion does not prevent the other rails from being measured.
    /// @param[in,out] measures Measures to update with the rails voltages, indexed like rails_
    /// @return 0 on success, negative errno code otherwise
    int read_voltages(etl::array<RailMeasure, RAILS_MAX>& measures);

    /// @brief Select the current mode and measure the current of every rail
    /// @details A failing conversion does not prevent the other rails from being measured.
    /// @param[in,out] measures Measures to update with the rails currents, indexed like rails_
    /// @return 0 on success, negative errno code otherwise
    int read_currents(etl::array<RailMeasure, RAILS_MAX>& measures);

    /// @brief Read the voltage on an ADC input
    /// @param[in]  channel ADC channel to read
    /// @param[out] adc_uv  ADC input voltage in microvolts
    /// @return 0 on success, negative errno code otherwise
    int read_adc_uv(max1161x_channel_t channel, uint32_t& adc_uv);

    const PowerMonitorParameters& parameters_;         ///< Power monitor parameters
    etl::span<const RailParameters> rails_;            ///< Monitored rails
    gpio::GPIO mode_gpio_;                             ///< Analog switch mode pin
    max1161x_t adc_ = {};                              ///< MAX1161X ADC device
    etl::array<RailMeasure, RAILS_MAX> measures_ = {}; ///< Last measures, indexed like rails_
    mutable mutex_t mutex_ = MUTEX_INIT;               ///< Protects measures_
};

} // namespace power_monitor
} // namespace cogip

/// @}
