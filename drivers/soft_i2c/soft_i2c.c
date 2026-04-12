// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     drivers_soft_i2c
/// @{
/// @file
/// @brief       Software I2C bitbanging implementation
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#include <assert.h>

#include "log.h"
#include "mutex.h"
#include "periph/gpio.h"
#include "ztimer.h"

#include "soft_i2c/soft_i2c.h"
#include "soft_i2c/soft_i2c_params.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/// Bus locks
static mutex_t locks[SOFT_I2C_NUMOF];

/// Half-period delay in microseconds per bus
static uint32_t half_period_us[SOFT_I2C_NUMOF];

// ============================================================================
// Low-level GPIO helpers (open-drain emulation)
// ============================================================================

static inline void scl_low(soft_i2c_t dev)
{
    gpio_init(soft_i2c_config[dev].scl_pin, GPIO_OUT);
    gpio_clear(soft_i2c_config[dev].scl_pin);
}

static inline void scl_high(soft_i2c_t dev)
{
    gpio_init(soft_i2c_config[dev].scl_pin, GPIO_IN_PU);
}

static inline int scl_read(soft_i2c_t dev)
{
    return gpio_read(soft_i2c_config[dev].scl_pin);
}

static inline void sda_low(soft_i2c_t dev)
{
    gpio_init(soft_i2c_config[dev].sda_pin, GPIO_OUT);
    gpio_clear(soft_i2c_config[dev].sda_pin);
}

static inline void sda_high(soft_i2c_t dev)
{
    gpio_init(soft_i2c_config[dev].sda_pin, GPIO_IN_PU);
}

static inline int sda_read(soft_i2c_t dev)
{
    return gpio_read(soft_i2c_config[dev].sda_pin);
}

static inline void delay(soft_i2c_t dev)
{
    if (half_period_us[dev] > 0) {
        ztimer_sleep(ZTIMER_USEC, half_period_us[dev]);
    }
}

static inline void wait_scl_high(soft_i2c_t dev)
{
    scl_high(dev);
    uint32_t timeout = 1000;
    while (!scl_read(dev) && timeout > 0) {
        timeout--;
    }
    if (timeout == 0) {
        LOG_WARNING("soft_i2c: clock stretching timeout\n");
    }
}

// ============================================================================
// I2C protocol primitives
// ============================================================================

static void i2c_start(soft_i2c_t dev)
{
    sda_high(dev);
    delay(dev);
    scl_high(dev);
    delay(dev);
    sda_low(dev);
    delay(dev);
    scl_low(dev);
    delay(dev);
}

static void i2c_stop(soft_i2c_t dev)
{
    sda_low(dev);
    delay(dev);
    wait_scl_high(dev);
    delay(dev);
    sda_high(dev);
    delay(dev);
}

static void i2c_write_bit(soft_i2c_t dev, int bit)
{
    if (bit) {
        sda_high(dev);
    } else {
        sda_low(dev);
    }
    delay(dev);
    wait_scl_high(dev);
    delay(dev);
    scl_low(dev);
}

static int i2c_read_bit(soft_i2c_t dev)
{
    sda_high(dev);
    delay(dev);
    wait_scl_high(dev);
    int bit = sda_read(dev) ? 1 : 0;
    delay(dev);
    scl_low(dev);
    return bit;
}

static int i2c_write_byte(soft_i2c_t dev, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        i2c_write_bit(dev, (byte >> i) & 1);
    }
    return i2c_read_bit(dev);
}

static uint8_t i2c_read_byte(soft_i2c_t dev, int ack)
{
    uint8_t byte = 0;
    for (int i = 7; i >= 0; i--) {
        byte |= (uint8_t)(i2c_read_bit(dev) << i);
    }
    i2c_write_bit(dev, ack ? 0 : 1);
    return byte;
}

// ============================================================================
// Public API
// ============================================================================

void soft_i2c_init(soft_i2c_t dev)
{
    assert(dev < SOFT_I2C_NUMOF);

    mutex_init(&locks[dev]);

    if (soft_i2c_config[dev].speed_hz > 0) {
        half_period_us[dev] = 500000UL / soft_i2c_config[dev].speed_hz;
    } else {
        half_period_us[dev] = 5;
    }

    scl_high(dev);
    sda_high(dev);

    // Bus clear: clock out 9 bits to release any stuck slave
    for (int i = 0; i < 9; i++) {
        scl_low(dev);
        delay(dev);
        scl_high(dev);
        delay(dev);
    }

    i2c_stop(dev);

    LOG_INFO("soft_i2c: bus %u initialized (half_period=%lu us)\n", dev,
             (unsigned long)half_period_us[dev]);
}

void soft_i2c_acquire(soft_i2c_t dev)
{
    assert(dev < SOFT_I2C_NUMOF);
    mutex_lock(&locks[dev]);
}

void soft_i2c_release(soft_i2c_t dev)
{
    assert(dev < SOFT_I2C_NUMOF);
    mutex_unlock(&locks[dev]);
}

int soft_i2c_read_regs(soft_i2c_t dev, uint8_t addr, uint8_t reg, uint8_t* data, size_t len)
{
    assert(dev < SOFT_I2C_NUMOF);

    i2c_start(dev);
    if (i2c_write_byte(dev, (uint8_t)(addr << 1))) {
        DEBUG("soft_i2c: NACK on address (write) 0x%02x\n", addr);
        i2c_stop(dev);
        return -1;
    }
    if (i2c_write_byte(dev, reg)) {
        DEBUG("soft_i2c: NACK on register 0x%02x\n", reg);
        i2c_stop(dev);
        return -1;
    }

    i2c_start(dev);
    if (i2c_write_byte(dev, (uint8_t)((addr << 1) | 1))) {
        DEBUG("soft_i2c: NACK on address (read) 0x%02x\n", addr);
        i2c_stop(dev);
        return -1;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(dev, (i < len - 1) ? 1 : 0);
    }

    i2c_stop(dev);
    return 0;
}

int soft_i2c_write_regs(soft_i2c_t dev, uint8_t addr, uint8_t reg, const uint8_t* data, size_t len)
{
    assert(dev < SOFT_I2C_NUMOF);

    i2c_start(dev);
    if (i2c_write_byte(dev, (uint8_t)(addr << 1))) {
        DEBUG("soft_i2c: NACK on address (write) 0x%02x\n", addr);
        i2c_stop(dev);
        return -1;
    }
    if (i2c_write_byte(dev, reg)) {
        DEBUG("soft_i2c: NACK on register 0x%02x\n", reg);
        i2c_stop(dev);
        return -1;
    }

    for (size_t i = 0; i < len; i++) {
        if (i2c_write_byte(dev, data[i])) {
            DEBUG("soft_i2c: NACK on data byte %u\n", (unsigned)i);
            i2c_stop(dev);
            return -1;
        }
    }

    i2c_stop(dev);
    return 0;
}

int soft_i2c_read_reg(soft_i2c_t dev, uint8_t addr, uint8_t reg, uint8_t* data)
{
    return soft_i2c_read_regs(dev, addr, reg, data, 1);
}

int soft_i2c_write_reg(soft_i2c_t dev, uint8_t addr, uint8_t reg, uint8_t data)
{
    return soft_i2c_write_regs(dev, addr, reg, &data, 1);
}

/// @}
