/*
 * Copyright (C) 2018 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip2018-cortex COGIP 2018 Cortex board
 * @ingroup     boards_cogip2018-cortex
 * @brief       Support for the COGIP 2018 Cortex board
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "motor_driver.h"
#include "vl53l0x.h"
#include "pca9548.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9548_SENSORS 0

/* Camp selection */
#define GPIO_CAMP       GPIO_PIN(PORT_B, 1)
/* Starting switch */
#define GPIO_STARTER    GPIO_PIN(PORT_B, 2)

/* Pumps */
#define GPIO_BL_PUMP_1  GPIO_PIN(PORT_C, 11)
#define GPIO_BC_PUMP_2  GPIO_PIN(PORT_A, 12)
#define GPIO_BR_PUMP_3  GPIO_PIN(PORT_A, 11)
#define GPIO_FL_PUMP_4  GPIO_PIN(PORT_A, 10)
#define GPIO_FC_PUMP_5  GPIO_PIN(PORT_A, 9)
#define GPIO_FR_PUMP_6  GPIO_PIN(PORT_B, 15)

#define GPIO_DEBUG_LED  GPIO_PIN(PORT_C, 8)

/**
 * @brief Describe DC motor with PWM channel and GPIOs
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .mode = MOTOR_DRIVER_1_DIR,
        .pwm_dev = 0,
        .pwm_frequency = 20000U,
        .pwm_resolution = 1500U,
        .nb_motors = 2,
        .motors = {
            {
                .pwm_channel = 0,
                .gpio_enable = GPIO_UNDEF,
                .gpio_dir0 = GPIO_PIN(PORT_A, 4),
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 0,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_UNDEF,
                .gpio_dir0 = GPIO_PIN(PORT_B, 8),
                .gpio_dir1_or_brake = GPIO_UNDEF,
                .gpio_dir_reverse = 1,
                .gpio_enable_invert = 0,
                .gpio_brake_invert = 0,
            },
        },
    },
};

#define MOTOR_DRIVER_NUMOF      (sizeof(motor_driver_config) / sizeof(motor_driver_config[0]))

static const vl53l0x_conf_t vl53l0x_config[] = {
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
    {
        .i2c_dev    = 1,
        .i2c_addr   = 0x29,
    },
};

#define VL53L0X_NUMOF     (sizeof(vl53l0x_config) / sizeof(vl53l0x_config[0]))

static const pca9548_conf_t pca9548_config[] = {
    {
        .i2c_dev_id         = 1,
        .i2c_address        = 0x70,
        .channel_numof      = PCA9548_CHANNEL_MAX,
    },
};

#define PCA9548_NUMOF      (sizeof(motor_driver_config) / sizeof(motor_driver_config[0]))

static const uint8_t vl53l0x_channel[VL53L0X_NUMOF] = {
    0,
    1,
    2,
    6,
    4,
    5,
};

#ifndef CC110X_DEFAULT_PATABLE
#define CC110X_DEFAULT_PATABLE cc110x_cortex_pa_table
extern const char cc110x_cortex_pa_table[8];
#endif

#ifndef CC110X_DEFAULT_FREQ
#define CC110X_DEFAULT_FREQ cc110x_cortex_base_freq
extern const char cc110x_cortex_base_freq[3];
#endif

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
