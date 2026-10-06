// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @file main.c
/// @brief Power supply board ADC example (MAX1161X driver and SAUL)

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "kernel_defines.h"
#include "log.h"
#include "max1161x.h"
#include "max1161x_params.h"
#include "max1161x_regs.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "saul_reg.h"
#include "shell.h"
#include "ztimer.h"

/// @brief ADC reference voltage in mV
#define ADC_VREF_MV (3300)

/// @brief ADC full scale in LSB (12-bit conversion)
#define ADC_FULL_SCALE (MAX1161X_DATA_MASK + 1)

/// @brief Mode selection pin
/// @details Low = current measurement, high = voltage measurement
#define ADC_V_I_EN_PIN GPIO_PIN(PORT_C, 13)

/// @brief Shunt resistor in mOhm
#define SHUNT_MOHM (10)

/// @brief MAX44284F current sense amplifier gain in V/V
#define CURRENT_SENSE_GAIN (50)

/// @brief Rail measurement channel and its voltage divider
typedef struct
{
    max1161x_channel_t chan; ///< MAX1161X ADC channel
    const char* name;        ///< Rail name
    uint32_t div_top_ohm;    ///< Voltage divider top resistor in ohm
    uint32_t div_bot_ohm;    ///< Voltage divider bottom resistor in ohm
} rail_t;

/// @brief Select the measurement mode and wait for the analog switch to settle
/// @param[in] current true to measure currents, false to measure voltages
static void _set_mode(bool current);

/// @brief SAUL read callback of a rail
/// @param[in] arg  Pointer to the @ref rail_t of the rail
/// @param[out] res Measured value: mV in voltage mode, mA in current mode
/// @return Number of values written to @p res, or a negative errno on ADC read error
static int _read_rail(const void* arg, phydat_t* res);

/// @brief SAUL read callback of the mode switch
/// @param[in]  arg Unused
/// @param[out] res Current mode: 1 = current measurement, 0 = voltage measurement
/// @return Number of values written to @p res
static int _read_mode(const void* arg, phydat_t* res);

/// @brief SAUL write callback of the mode switch
/// @param[in] arg  Unused
/// @param[in] data Mode to select: non-zero = current measurement, 0 = voltage measurement
/// @return Number of values consumed from @p data
static int _write_mode(const void* arg, const phydat_t* data);

/// @brief SAUL driver of the rail measurements (read only)
static const saul_driver_t rail_driver = {
    .read = _read_rail,
    .write = saul_write_notsup,
    .type = SAUL_SENSE_ANALOG,
};

/// @brief SAUL driver of the voltage/current mode switch
static const saul_driver_t mode_driver = {
    .read = _read_mode,
    .write = _write_mode,
    .type = SAUL_ACT_SWITCH,
};

/// @brief Measured rails, indexed like @ref rail_entries
static const rail_t rails[] = {
    {MAX1161X_CHANNEL_CH0, "PxVx", 10000, 10000}, {MAX1161X_CHANNEL_CH1, "P7V5", 10000, 10000},
    {MAX1161X_CHANNEL_CH2, "P5V0", 10000, 10000}, {MAX1161X_CHANNEL_CH3, "P3V3", 10000, 10000},
    {MAX1161X_CHANNEL_CH4, "P12V0", 10000, 3900},
};

/// @brief SAUL registry entries of the rails
static saul_reg_t rail_entries[] = {
    {.name = "PxVx", .dev = (void*)&rails[0], .driver = &rail_driver},
    {.name = "P7V5", .dev = (void*)&rails[1], .driver = &rail_driver},
    {.name = "P5V0", .dev = (void*)&rails[2], .driver = &rail_driver},
    {.name = "P3V3", .dev = (void*)&rails[3], .driver = &rail_driver},
    {.name = "P12V0", .dev = (void*)&rails[4], .driver = &rail_driver},
};

/// @brief SAUL registry entry of the mode switch
static saul_reg_t mode_entry = {
    .name = "ADC_V_I_EN",
    .driver = &mode_driver,
};

/// @brief MAX1161X ADC device
static max1161x_t dev;

/// @brief Current measurement mode
static bool current_mode;

int main(void)
{
    LOG_INFO("Power supply board ADC example\n\n");

    /* I2C is not initialized automatically on cogip-board */
    i2c_init(max1161x_params[0].i2c);

    gpio_init(ADC_V_I_EN_PIN, GPIO_OUT);
    _set_mode(false);

    int ret = max1161x_init(&dev, &max1161x_params[0]);
    if (ret < 0) {
        LOG_ERROR("MAX1161X init failed (%d), check I2C wiring and address 0x%02x\n", ret,
                  max1161x_params[0].addr);
        return 1;
    }

    for (unsigned i = 0; i < ARRAY_SIZE(rail_entries); i++) {
        saul_reg_add(&rail_entries[i]);
    }

    saul_reg_add(&mode_entry);

    LOG_INFO("Type `saul` to list the devices, `saul read <id>` to read a rail,\n");
    LOG_INFO("`saul write <ADC_V_I_EN id> 0|1` to select voltage or current mode.\n\n");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}

static void _set_mode(bool current)
{
    current_mode = current;
    gpio_write(ADC_V_I_EN_PIN, !current);
    ztimer_sleep(ZTIMER_MSEC, 10);
}

static int _read_rail(const void* arg, phydat_t* res)
{
    const rail_t* rail = arg;
    int16_t raw;

    int ret = max1161x_read_channel_raw(&dev, rail->chan, &raw);
    if (ret < 0) {
        LOG_ERROR("%s: read error (%d)\n", rail->name, ret);
        return -ECANCELED;
    }

    /* ADC input voltage in uV */
    const int64_t vadc_uv = (int64_t)raw * ADC_VREF_MV * 1000 / ADC_FULL_SCALE;

    if (current_mode) {
        /* I(mA) = Vadc(uV) / (gain * shunt(mOhm)) */
        res->val[0] = vadc_uv / (CURRENT_SENSE_GAIN * SHUNT_MOHM);
        res->unit = UNIT_A;
    } else {
        /* V(mV) = Vadc * (top + bottom) / bottom */
        res->val[0] = vadc_uv * (rail->div_top_ohm + rail->div_bot_ohm) / rail->div_bot_ohm / 1000;
        res->unit = UNIT_V;
    }
    res->scale = -3;

    return 1;
}

static int _read_mode(const void* arg, phydat_t* res)
{
    (void)arg;

    res->val[0] = current_mode;
    res->unit = UNIT_BOOL;
    res->scale = 0;

    return 1;
}

static int _write_mode(const void* arg, const phydat_t* data)
{
    (void)arg;

    _set_mode(data->val[0] != 0);

    return 1;
}
