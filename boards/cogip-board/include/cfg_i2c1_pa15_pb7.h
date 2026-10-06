/*
 * Copyright (C) 2026 COGIP Robotics association
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
 * @brief       I2C1 configuration for cogip-board
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 */

#ifndef CFG_I2C1_PA15_PB7_H
#define CFG_I2C1_PA15_PB7_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {{
    .dev = I2C1,
    .speed = I2C_SPEED_NORMAL,
    .scl_pin = GPIO_PIN(PORT_A, 15),
    .sda_pin = GPIO_PIN(PORT_B, 7),
    .scl_af = GPIO_AF4,
    .sda_af = GPIO_AF4,
    .bus = APB1,
    .rcc_mask = RCC_APB1ENR1_I2C1EN,
    .rcc_sw_mask = RCC_CCIPR_I2C1SEL_1, /* HSI (16 MHz) */
    .irqn = I2C1_ER_IRQn,
}};

#define I2C_0_ISR isr_i2c1_er

#define I2C_NUMOF ARRAY_SIZE(i2c_config)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CFG_I2C1_PA15_PB7_H */
/** @} */
