/*
 * Copyright (C) 2021 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip-nucleo-f446re
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the COGIP test board (based on nucleo-f446re)
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* This board provides an LSE */
#ifndef CONFIG_BOARD_HAS_LSE
#define CONFIG_BOARD_HAS_LSE    1
#endif

/* This board provides an HSE */
#ifndef CONFIG_BOARD_HAS_HSE
#define CONFIG_BOARD_HAS_HSE    1
#endif

#include "periph_cpu.h"
#include "clk_conf.h"
#include "cfg_i2c1_pb8_pb9.h"
#include "cfg_timer_tim5.h"

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
        .dev = UART4,
        .rcc_mask = RCC_APB1ENR_UART4EN,
        .rx_pin = GPIO_PIN(PORT_A, 1),
        .tx_pin = GPIO_PIN(PORT_A, 0),
        .rx_af = GPIO_AF8,
        .tx_af = GPIO_AF8,
        .bus = APB1,
        .irqn = UART4_IRQn,
#ifdef MODULE_PERIPH_DMA
        .dma = DMA_STREAM_UNDEF,
        .dma_chan = UINT8_MAX,
#endif
    },
};

#define UART_0_ISR          (isr_usart2)
#define UART_1_ISR          (isr_uart4)
#define UART_2_ISR          (isr_usart3)

#define UART_NUMOF          ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev = TIM2,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .chan = { { .pin = GPIO_PIN(PORT_A, 15), .cc_chan = 0 },
                  { .pin = GPIO_PIN(PORT_B,  3), .cc_chan = 1 },
                  { .pin = GPIO_PIN(PORT_B, 10), .cc_chan = 2 },
                  { .pin = GPIO_PIN(PORT_B,  2), .cc_chan = 3 } },
        .af = GPIO_AF1,
        .bus = APB1
    },
    {
        .dev = TIM8,
        .rcc_mask = RCC_APB2ENR_TIM8EN,
        .chan = { { .pin = GPIO_PIN(PORT_C, 6), .cc_chan = 0 },
                  { .pin = GPIO_PIN(PORT_C, 7), .cc_chan = 1 },
                  { .pin = GPIO_PIN(PORT_C, 8), .cc_chan = 2 },
                  { .pin = GPIO_PIN(PORT_C, 9), .cc_chan = 3 } },
        .af = GPIO_AF3,
        .bus = APB2
    },
};

#define PWM_NUMOF           ARRAY_SIZE(pwm_config)
/** @} */

/**
 * @name    QDEC configuration
 * @{
 */
static const qdec_conf_t qdec_config[] = {
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
    {
        .dev = TIM3,
        .max = 0xffffffff,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .chan = { { .pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0 },
                  { .pin = GPIO_PIN(PORT_A, 7), .cc_chan = 1 } },
        .af = GPIO_AF2,
        .bus = APB1,
        .irqn = TIM3_IRQn
    }
};

#define QDEC_0_ISR         isr_tim3
#define QDEC_1_ISR         isr_tim4

#define QDEC_NUMOF           ARRAY_SIZE(qdec_config)
/** @} */

/**
 * @name   ADC configuration
 *
 * Note that we do not configure all ADC channels,
 * and not in the STM32F446 order.  Instead, we
 * just define 6 ADC channels, for the Nucleo
 * Arduino header pins A0-A5
 *
 * @{
 */
static const adc_conf_t adc_config[] = {
    { GPIO_PIN(PORT_A, 0), 0, 0 },
    { GPIO_PIN(PORT_A, 1), 0, 1 },
    { GPIO_PIN(PORT_A, 4), 0, 4 },
    { GPIO_PIN(PORT_B, 0), 0, 8 },
    { GPIO_PIN(PORT_C, 1), 0, 11 },
    { GPIO_PIN(PORT_C, 0), 0, 10 },
};

#define ADC_NUMOF           ARRAY_SIZE(adc_config)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
