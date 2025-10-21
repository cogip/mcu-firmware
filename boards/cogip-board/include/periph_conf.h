/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip-board
 * @brief       Support for the COGIP 2024 cogip-board
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the cogip-board board
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

/* Add specific clock configuration (HSE, LSE) for this board here */
#define CONFIG_BOARD_HAS_LSE 0
/* This board provides a 24MHz HSE oscillator */
#define CONFIG_BOARD_HAS_HSE 1
/* By default, configure a 80MHz SYSCLK with PLL using HSE as input clock */
#define CONFIG_CLOCK_PLL_M (6)

#include "cfg_i2c2_pa9_pa8.h"
#include "cfg_rtt_default.h"
#include "cfg_timer_tim2.h"
#include "clk_conf.h"
#include "mtd_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        /* Console */
        .dev = LPUART1,
        .rcc_mask = RCC_APB1ENR2_LPUART1EN,
        .rx_pin = GPIO_PIN(PORT_A, 3),
        .tx_pin = GPIO_PIN(PORT_A, 2),
        .rx_af = GPIO_AF12,
        .tx_af = GPIO_AF12,
        .bus = APB12,
        .irqn = LPUART1_IRQn,
        .type = STM32_LPUART,
        .clk_src = 0,
    },
    {
        /* LX Servo */
        .dev = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin = GPIO_PIN(PORT_C, 5),
        .tx_pin = GPIO_PIN(PORT_C, 4),
        .rx_af = GPIO_AF7,
        .tx_af = GPIO_AF7,
        .bus = APB2,
        .irqn = USART1_IRQn,
        .type = STM32_USART,
        .clk_src = 0,
    },
};

#define UART_0_ISR (isr_lpuart1)
#define UART_1_ISR (isr_usart1)

#define UART_NUMOF ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    /* Motion motors */
    {.dev = TIM1,
     .rcc_mask = RCC_APB2ENR_TIM1EN,
     .chan =
         {/* Left motor PWM */
          {.pin = GPIO_PIN(PORT_A, 8), .cc_chan = 0},
          /* Right motor PWM */
          {.pin = GPIO_PIN(PORT_A, 9), .cc_chan = 1}},
     .af = GPIO_AF6,
     .bus = APB2},
};

#define PWM_NUMOF (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @name    QDEC configuration
 * @{
 */
static const qdec_conf_t qdec_config[] = {
    /* Left encoder */
    {.dev = TIM4,
     .max = 0xffffffff,
     .rcc_mask = RCC_APB1ENR1_TIM4EN,
     .chan = {{.pin = GPIO_PIN(PORT_B, 6), .cc_chan = 0},
              {.pin = GPIO_PIN(PORT_B, 7), .cc_chan = 1}},
     .af = GPIO_AF2,
     .bus = APB1,
     .irqn = TIM4_IRQn},
    /* Right encoder */
    {.dev = TIM3,
     .max = 0xffffffff,
     .rcc_mask = RCC_APB1ENR1_TIM3EN,
     .chan = {{.pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0},
              {.pin = GPIO_PIN(PORT_A, 4), .cc_chan = 1}},
     .af = GPIO_AF2,
     .bus = APB1,
     .irqn = TIM3_IRQn},
};

#define QDEC_0_ISR isr_tim3
#define QDEC_1_ISR isr_tim4

#define QDEC_NUMOF (sizeof(qdec_config) / sizeof(qdec_config[0]))
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
