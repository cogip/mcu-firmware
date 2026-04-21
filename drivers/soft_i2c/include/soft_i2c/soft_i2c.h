// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    drivers_soft_i2c Software I2C
/// @ingroup     drivers
/// @brief       Software I2C master using GPIO bitbanging
/// @details
///   Provides I2C master functionality on arbitrary GPIO pins. Pins are
///   driven in open-drain mode with external pull-ups required.
///   Configuration is defined in periph_conf.h like other RIOT peripherals.
/// @{
/// @file
/// @author      Gilles DOFFE <g.doffe@gmail.com>

#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stddef.h>
#include <stdint.h>

#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Software I2C device index type
typedef unsigned int soft_i2c_t;

/// @brief Device access macro
#ifndef SOFT_I2C_DEV
#define SOFT_I2C_DEV(x) (x)
#endif

/// @brief Software I2C bus configuration
typedef struct
{
    gpio_t scl_pin;    ///< SCL (clock) GPIO pin
    gpio_t sda_pin;    ///< SDA (data) GPIO pin
    uint32_t speed_hz; ///< Bus speed in Hz (e.g. 100000 for 100kHz)
} soft_i2c_conf_t;

/// @brief Initialize the software I2C bus
/// @param dev Bus index
void soft_i2c_init(soft_i2c_t dev);

/// @brief Acquire exclusive access to the bus
/// @param dev Bus index
void soft_i2c_acquire(soft_i2c_t dev);

/// @brief Release the bus
/// @param dev Bus index
void soft_i2c_release(soft_i2c_t dev);

/// @brief Read bytes from a device register
/// @param dev   Bus index
/// @param addr  7-bit I2C slave address
/// @param reg   Register address to read from
/// @param data  Buffer to read into
/// @param len   Number of bytes to read
/// @return 0 on success, negative on error
int soft_i2c_read_regs(soft_i2c_t dev, uint8_t addr, uint8_t reg, uint8_t* data, size_t len);

/// @brief Write bytes to a device register
/// @param dev   Bus index
/// @param addr  7-bit I2C slave address
/// @param reg   Register address to write to
/// @param data  Data to write
/// @param len   Number of bytes to write
/// @return 0 on success, negative on error
int soft_i2c_write_regs(soft_i2c_t dev, uint8_t addr, uint8_t reg, const uint8_t* data, size_t len);

/// @brief Read a single byte from a device register
/// @param dev   Bus index
/// @param addr  7-bit I2C slave address
/// @param reg   Register address
/// @param data  Pointer to store the byte
/// @return 0 on success, negative on error
int soft_i2c_read_reg(soft_i2c_t dev, uint8_t addr, uint8_t reg, uint8_t* data);

/// @brief Write a single byte to a device register
/// @param dev   Bus index
/// @param addr  7-bit I2C slave address
/// @param reg   Register address
/// @param data  Byte to write
/// @return 0 on success, negative on error
int soft_i2c_write_reg(soft_i2c_t dev, uint8_t addr, uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_I2C_H */

/// @}
