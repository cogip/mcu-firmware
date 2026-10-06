// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

#include "power_monitor/PowerMonitor.hpp"

// RIOT includes
#include "max1161x_regs.h"
#include "ztimer.h"

namespace cogip {
namespace power_monitor {

/// ADC full scale in LSB (12-bit conversion)
static constexpr uint64_t ADC_FULL_SCALE = MAX1161X_DATA_MASK + 1;

int PowerMonitor::init()
{
    int ret = gpio_init(parameters_.mode_pin, GPIO_OUT);
    if (ret) {
        return ret;
    }

    select_mode(Mode::VOLTAGE);

    return max1161x_init(&adc_, &parameters_.adc_params);
}

int PowerMonitor::update()
{
    etl::array<RailMeasure, RAILS_MAX> measures = {};
    uint32_t adc_uv;
    int ret;

    select_mode(Mode::VOLTAGE);
    for (size_t i = 0; i < rails_.size(); i++) {
        const RailParameters& rail = rails_[i];

        ret = read_adc_uv(rail.channel, adc_uv);
        if (ret != MAX1161X_OK) {
            return ret;
        }

        // V(mV) = Vadc(uV) * (top + bottom) / bottom / 1000
        measures[i].voltage_mv = static_cast<uint64_t>(adc_uv) *
                                 (rail.divider_top_ohm + rail.divider_bottom_ohm) /
                                 rail.divider_bottom_ohm / 1000;
    }

    select_mode(Mode::CURRENT);
    for (size_t i = 0; i < rails_.size(); i++) {
        const RailParameters& rail = rails_[i];

        ret = read_adc_uv(rail.channel, adc_uv);
        if (ret != MAX1161X_OK) {
            return ret;
        }

        // I(mA) = Vadc(uV) / (gain * shunt(mOhm))
        measures[i].current_ma = adc_uv / (rail.current_sense_gain * rail.shunt_mohm);
    }

    mutex_lock(&mutex_);
    measures_ = measures;
    mutex_unlock(&mutex_);

    return MAX1161X_OK;
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
    gpio_write(parameters_.mode_pin, mode == Mode::VOLTAGE);
    ztimer_sleep(ZTIMER_MSEC, parameters_.mode_settling_time_ms);
}

int PowerMonitor::read_adc_uv(max1161x_channel_t channel, uint32_t& adc_uv)
{
    int16_t raw;

    int ret = max1161x_read_channel_raw(&adc_, channel, &raw);
    if (ret != MAX1161X_OK) {
        return ret;
    }

    // Unipolar conversions results are never negative
    adc_uv = static_cast<uint64_t>(raw) * parameters_.adc_vref_mv * 1000 / ADC_FULL_SCALE;

    return MAX1161X_OK;
}

} // namespace power_monitor
} // namespace cogip
