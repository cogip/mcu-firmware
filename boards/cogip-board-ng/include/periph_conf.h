/*
 * Copyright (C) 2023 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-board COGIP 2018 Cortex board
 * @ingroup     boards_cogip-board
 * @brief       Support for the COGIP 2018 Cortex board
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the cogip-board board
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

#include "cfg_timer_tim5.h"
#include "clk_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    DMA streams configuration
 * @{
 */
static const dma_conf_t dma_config[] = {
    { .stream = 11 },   /* DMA2 Stream 3 - SPI1_TX */
    { .stream = 10 },   /* DMA2 Stream 2 - SPI1_RX */
    { .stream = 4 },    /* DMA1 Stream 4 - SPI2_TX */
    { .stream = 3 },    /* DMA1 Stream 3 - SPI2_RX */
    { .stream = 5 },    /* DMA1 Stream 5 - SPI3_TX */
    { .stream = 0 },    /* DMA1 Stream 0 - SPI3_RX */
};

#define DMA_0_ISR           isr_dma2_stream3
#define DMA_1_ISR           isr_dma2_stream2
#define DMA_2_ISR           isr_dma1_stream4
#define DMA_3_ISR           isr_dma1_stream3
#define DMA_4_ISR           isr_dma1_stream5
#define DMA_5_ISR           isr_dma1_stream0

#define DMA_NUMOF           ARRAY_SIZE(dma_config)
/** @} */

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        /* Console */
        .dev = USART2,
        .rcc_mask = RCC_APB1ENR_USART2EN,
        .rx_pin = GPIO_PIN(PORT_A, 3),
        .tx_pin = GPIO_PIN(PORT_A, 2),
        .rx_af = GPIO_AF7,
        .tx_af = GPIO_AF7,
        .bus = APB1,
        .irqn = USART2_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma = DMA_STREAM_UNDEF,
        .dma_chan = UINT8_MAX,
#endif
    },
    {
        /* Protobuf */
        .dev = USART1,
        .rcc_mask = RCC_APB2ENR_USART1EN,
        .rx_pin = GPIO_PIN(PORT_A, 10),
        .tx_pin = GPIO_PIN(PORT_A, 9),
        .rx_af = GPIO_AF7,
        .tx_af = GPIO_AF7,
        .bus = APB2,
        .irqn = USART1_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma = DMA_STREAM_UNDEF,
        .dma_chan = UINT8_MAX,
#endif
    },
    {
        /* Servomotors */
        .dev = UART5,
        .rcc_mask = RCC_APB1ENR_UART5EN,
        .rx_pin = GPIO_PIN(PORT_D, 2),
        .tx_pin = GPIO_PIN(PORT_C, 12),
        .rx_af = GPIO_AF8,
        .tx_af = GPIO_AF8,
        .bus = APB1,
        .irqn = UART5_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma = DMA_STREAM_UNDEF,
        .dma_chan = UINT8_MAX,
#endif
    }
};

#define UART_0_ISR      (isr_usart2)
#define UART_1_ISR      (isr_usart1)
#define UART_2_ISR      (isr_uart5)

#define UART_NUMOF      ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev = TIM2,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .chan = {
            /* Left motor PWM */
            { .pin = GPIO_PIN(PORT_A, 1), .cc_chan = 1 },
            /* Right motor PWM */
            { .pin = GPIO_PIN(PORT_A, 5), .cc_chan = 0 }
        },
        .af = GPIO_AF1,
        .bus = APB1
    },
};

#define PWM_NUMOF   (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @name    QDEC configuration
 * @{
 */
static const qdec_conf_t qdec_config[] = {
    /* Left encoder */
    {
        .dev = TIM4,
        .max = 0xffffffff,
        .rcc_mask = RCC_APB1ENR_TIM4EN,
        .chan = { { .pin = GPIO_PIN(PORT_B, 6), .cc_chan = 0 },
                  { .pin = GPIO_PIN(PORT_B, 7), .cc_chan = 1 } },
        .af = GPIO_AF2,
        .bus = APB1,
        .irqn = TIM4_IRQn
    },
    /* Right encoder */
    {
        .dev = TIM3,
        .max = 0xffffffff,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .chan = { { .pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0 },
                  { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 1 } },
        .af = GPIO_AF2,
        .bus = APB1,
        .irqn = TIM3_IRQn
    },
};

#define QDEC_0_ISR  isr_tim3
#define QDEC_1_ISR  isr_tim4

#define QDEC_NUMOF  (sizeof(qdec_config) / sizeof(qdec_config[0]))
/** @} */

/**
 * @name I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev = I2C1,
        .speed = I2C_SPEED_NORMAL,
        .scl_pin = GPIO_PIN(PORT_B, 8),
        .sda_pin = GPIO_PIN(PORT_B, 9),
        .scl_af = GPIO_AF4,
        .sda_af = GPIO_AF4,
        .bus = APB1,
        .rcc_mask = RCC_APB1ENR_I2C1EN,
        .clk = CLOCK_APB1,
        .irqn = I2C1_EV_IRQn
    },
};

#define I2C_0_ISR   isr_i2c1_ev

#define I2C_NUMOF   ARRAY_SIZE(i2c_config)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
