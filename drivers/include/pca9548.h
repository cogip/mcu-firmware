/*
 * Copyright (C) 2019 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    I2C switch PCA9548 driver
 * @ingroup     drivers
 * @brief       I2C switch PCA9548 driver
 *
 * The PCA9548 is an 8 channels I2C switch. It allows to drive up to 8 I2C
 * devices that shares the same I2C address.
 *
 * The PCA9548 has a default address of 0x70 defined on 7 bits.
 * Using three hardware pins, I2C address can be steup between 0x70 and 0x77.
 *
 * @{
 * @file
 * @brief       Common controllers API and datas
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef PCA9548_H_
#define PCA9548_H_

/* RIOT includes */
#include "periph/i2c.h"

#define PCA9548_CHANNEL_MAX  8
#define PCA9548_DEFAULT_CHANNEL 0

/**
 * @brief   PCA9548 id
 */
typedef unsigned int pca9548_t;

/**
 * @brief   PCA9548 configuration
 */
typedef struct {
    i2c_t i2c_dev_id;                       /**< I2C bus */
    uint16_t i2c_address;                   /**< I2C board address */
    i2c_speed_t i2c_speed_khz;              /**< I2C bus speed */
    uint8_t channel_numof;
} pca9548_conf_t;


/**
 * @brief Initialize PCA9548 driver according to static configuration.
 *
 * @return
 */
void pca9548_init(void);

/**
 * @brief Set current channel
 *
 * @param[in]   dev             PCA9548 device id
 * @param[in]   channel         New channel
 *
 * @return
 */
void pca9548_set_current_channel(pca9548_t dev, uint8_t channel);

/**
 * @brief Get current channel
 *
 * @param[in]   dev     PCA9548 device id
 *
 * @return              PCA9548 current channel
 */
uint8_t pca9548_get_current_channel(pca9548_t dev);

#endif /* PCA9548_H_ */
/** @} */
