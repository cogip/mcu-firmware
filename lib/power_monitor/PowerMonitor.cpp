// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "power_monitor/PowerMonitor.hpp"

#include <cerrno>

// RIOT includes
#include "max1161x_regs.h"
#include "ztimer.h"

// ETL includes
#include "etl/algorithm.h"

namespace cogip {
namespace power_monitor {

/// ADC full scale in LSB (12-bit conversion)
static constexpr uint64_t ADC_FULL_SCALE = MAX1161X_DATA_MASK + 1;

/// Rails parameters are used as divisors when computing the measures
static bool has_null_divisor(const RailParameters& rail)
{
    return rail.divider_bottom_ohm == 0 || rail.shunt_mohm == 0 || rail.current_sense_gain == 0;
}

int PowerMonitor::init()
{
    if (etl::any_of(rails_.begin(), rails_.end(), has_null_divisor)) {
        return -EINVAL;
    }

    select_mode(Mode::VOLTAGE);

    return max1161x_init(&adc_, &parameters_.adc_params);
}

int PowerMonitor::update()
{
    // Start from the last measures so a failing channel keeps its previous value
    mutex_lock(&mutex_);
    etl::array<RailMeasure, RAILS_MAX> measures = measures_;
    mutex_unlock(&mutex_);

    int voltages_ret = read_voltages(measures);
    int currents_ret = read_currents(measures);

    mutex_lock(&mutex_);
    measures_ = measures;
    mutex_unlock(&mutex_);

    return voltages_ret ? voltages_ret : currents_ret;
}

int PowerMonitor::read_voltages(etl::array<RailMeasure, RAILS_MAX>& measures)
{
    int result = 0;
    uint32_t adc_uv;

    select_mode(Mode::VOLTAGE);
    for (size_t i = 0; i < rails_.size(); i++) {
        const RailParameters& rail = rails_[i];

        int ret = read_adc_uv(rail.channel, adc_uv);
        if (ret) {
            result = result ? result : ret;
            continue;
        }

        // V(mV) = Vadc(uV) * (top + bottom) / bottom / 1000
        measures[i].voltage_mv = static_cast<uint64_t>(adc_uv) *
                                 (rail.divider_top_ohm + rail.divider_bottom_ohm) /
                                 rail.divider_bottom_ohm / 1000;
    }

    return result;
}

int PowerMonitor::read_currents(etl::array<RailMeasure, RAILS_MAX>& measures)
{
    int result = 0;
    uint32_t adc_uv;

    select_mode(Mode::CURRENT);
    for (size_t i = 0; i < rails_.size(); i++) {
        const RailParameters& rail = rails_[i];

        int ret = read_adc_uv(rail.channel, adc_uv);
        if (ret) {
            result = result ? result : ret;
            continue;
        }

        // I(mA) = Vadc(uV) / (gain * shunt(mOhm))
        measures[i].current_ma = adc_uv / (rail.current_sense_gain * rail.shunt_mohm);
    }

    return result;
}

RailMeasure PowerMonitor::measure(size_t index) const
{
    mutex_lock(&mutex_);
    RailMeasure measure = measures_[index];
    mutex_unlock(&mutex_);

    return measure;
}

void PowerMonitor::select_mode(Mode mode)
{
    mode_gpio_.write(mode == Mode::VOLTAGE);
    ztimer_sleep(ZTIMER_MSEC, parameters_.mode_settling_time_ms);
}

int PowerMonitor::read_adc_uv(max1161x_channel_t channel, uint32_t& adc_uv)
{
    int16_t raw;

    int ret = max1161x_read_channel_raw(&adc_, channel, &raw);
    if (ret) {
        return ret;
    }

    // Unipolar conversions results are never negative
    adc_uv = static_cast<uint64_t>(raw) * parameters_.adc_vref_mv * 1000 / ADC_FULL_SCALE;

    return 0;
}

} // namespace power_monitor
} // namespace cogip
