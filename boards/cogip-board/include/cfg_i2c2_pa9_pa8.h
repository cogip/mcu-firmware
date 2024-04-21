/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip_board
 * @{
 *
 * @file
 * @brief       Common configuration for cogip-board STM32 I2C
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef CFG_I2C1_PA9_PA8_H
#define CFG_I2C1_PA9_PA8_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev = I2C2,
        .speed = I2C_SPEED_NORMAL,
        .scl_pin = GPIO_PIN(PORT_A, 9),
        .sda_pin = GPIO_PIN(PORT_A, 8),
        .scl_af = GPIO_AF4,
        .sda_af = GPIO_AF4,
        .bus = APB1,
        .rcc_mask = RCC_APB1ENR1_I2C2EN,
        .rcc_sw_mask = RCC_CCIPR_I2C2SEL_1,             /* HSI (16 MHz) */
        .irqn = I2C2_ER_IRQn,
    }
};

#define I2C_0_ISR           isr_i2c2_er

#define I2C_NUMOF           ARRAY_SIZE(i2c_config)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CFG_I2C2_PA9_PA8_H */
/** @} */
