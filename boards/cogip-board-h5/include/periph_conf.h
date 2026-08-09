/*
 * Copyright (C) 2026 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip-board-h5
 * @brief       Support for the COGIP 2026 CAN+ETH module (STM32H563RITx)
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the cogip-board-h5 board
 *
 * Pin assignments are taken from the KiCad schematic net names (authoritative)
 * of the 2026 stm32-can-eth-module and the CubeMX stm32_eth.ioc peripheral
 * intent. The MCU is clocked by a 25 MHz HSE crystal (PH0/PH1) and a
 * 32.768 kHz LSE crystal (PC14/PC15).
 *
 * Alternate function numbers follow the STM32H563 datasheet (DS14258):
 * TIM1 = AF1, TIM3/TIM4/TIM5 = AF2, USART3 = AF7, FDCAN1 = AF9, ETH = AF11.
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

/* 25 MHz HSE crystal + 32.768 kHz LSE crystal on this board */
#ifndef CONFIG_BOARD_HAS_LSE
#define CONFIG_BOARD_HAS_LSE        1
#endif
#ifndef CONFIG_BOARD_HAS_HSE
#define CONFIG_BOARD_HAS_HSE        1
#endif
#ifndef CONFIG_CLOCK_HSE
#define CONFIG_CLOCK_HSE            MHZ(25)
#endif

#include "periph_cpu.h"
#include "clk_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM2,
        .max      = 0xffffffff,
        .rcc_mask = RCC_APB1LENR_TIM2EN,
        .bus      = APB1,
        .irqn     = TIM2_IRQn,
    },
};

#define TIMER_0_ISR         isr_tim2

#define TIMER_NUMOF         ARRAY_SIZE(timer_config)
/** @} */

/**
 * @name    UART configuration
 *
 * Console / debug UART broken out on the DBG_UART header:
 * PC10 = DBG_UART_MCU_TO_EXT (TX), PC11 = DBG_UART_EXT_TO_MCU (RX) -> USART3.
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev        = USART3,
        .rcc_mask   = RCC_APB1LENR_USART3EN,
        .rx_pin     = GPIO_PIN(PORT_C, 11),
        .tx_pin     = GPIO_PIN(PORT_C, 10),
        .rx_af      = GPIO_AF7,
        .tx_af      = GPIO_AF7,
        .bus        = APB1,
        .irqn       = USART3_IRQn,
    },
};

#define UART_0_ISR          (isr_usart3)

#define UART_NUMOF          ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name    PWM configuration
 *
 * Motion motor PWM on TIM1: PA8 = CH1 (left), PA9 = CH2 (right).
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM1,
        .rcc_mask = RCC_APB2ENR_TIM1EN,
        .chan     = {
            {.pin = GPIO_PIN(PORT_A, 8), .cc_chan = 0},   /* left motor PWM  */
            {.pin = GPIO_PIN(PORT_A, 9), .cc_chan = 1},   /* right motor PWM */
            {.pin = GPIO_UNDEF, .cc_chan = 0},
            {.pin = GPIO_UNDEF, .cc_chan = 0},
        },
        .af       = GPIO_AF1,
        .bus      = APB2,
    },
};

#define PWM_NUMOF           ARRAY_SIZE(pwm_config)
/** @} */

/**
 * @name    QDEC configuration
 *
 * Left encoder  : TIM4, PB6 = CH1 (A), PB7 = CH2 (B).
 *                 Optional hardware index on PA4 (TIM5_ETR) routed into TIM4
 *                 via TIM4_AF1.ETRSEL = 0b1011 (tim_etr11, RM0481 Table 422);
 *                 the remap is a firmware register write, not configured here.
 * Right encoder : TIM3, PA6 = CH1 (A), PC7 = CH2 (B).
 *                 Optional hardware index on PD2 (TIM3_ETR).
 * @{
 */
static const qdec_conf_t qdec_config[] = {
    /* Left encoder */
    {.dev = TIM4,
     .max = 0xffff,
     .rcc_mask = RCC_APB1LENR_TIM4EN,
     .chan = {{.pin = GPIO_PIN(PORT_B, 6), .cc_chan = 0},
              {.pin = GPIO_PIN(PORT_B, 7), .cc_chan = 1}},
     .af = GPIO_AF2,
     .bus = APB1,
     .irqn = TIM4_IRQn},
    /* Right encoder */
    {.dev = TIM3,
     .max = 0xffff,
     .rcc_mask = RCC_APB1LENR_TIM3EN,
     .chan = {{.pin = GPIO_PIN(PORT_A, 6), .cc_chan = 0},
              {.pin = GPIO_PIN(PORT_C, 7), .cc_chan = 1}},
     .af = GPIO_AF2,
     .bus = APB1,
     .irqn = TIM3_IRQn},
};

#define QDEC_0_ISR isr_tim4
#define QDEC_1_ISR isr_tim3

#define QDEC_NUMOF ARRAY_SIZE(qdec_config)
/** @} */

/**
 * @name    Ethernet configuration
 *
 * LAN8742A 10/100 PHY wired to the MAC in RMII mode. Pin map from the 2026
 * schematic net names. FDCAN clock aside, all RMII lines are on AF11 (set
 * internally by the stm32_eth driver).
 * @{
 */
#include "mii.h"

static const eth_conf_t eth_config = {
    .mode = RMII,
    .speed = MII_BMCR_SPEED_100 | MII_BMCR_FULL_DPLX,
    .dma = 0,
    .dma_chan = 0,
    .phy_addr = 0x00,
    .pins = {
        GPIO_PIN(PORT_B, 12),       /* RMII_TXD0     */
        GPIO_PIN(PORT_B, 15),       /* RMII_TXD1     */
        GPIO_PIN(PORT_A, 5),        /* RMII_TX_EN    */
        GPIO_PIN(PORT_C, 4),        /* RMII_RXD0     */
        GPIO_PIN(PORT_C, 5),        /* RMII_RXD1     */
        GPIO_PIN(PORT_A, 7),        /* RMII_CRS_DV   */
        GPIO_PIN(PORT_C, 1),        /* ETH_MDC       */
        GPIO_PIN(PORT_A, 2),        /* ETH_MDIO      */
        GPIO_PIN(PORT_A, 1),        /* RMII_REF_CLK  */
    }
};
/** @} */

/*
 * FDCAN1 (PA11 = CAN_RXD, PA12 = CAN_TXD, AF9) is configured in the
 * board-local include/can_params.h, which shadows the cpu-level default.
 */

#ifdef __cplusplus
}
#endif

/** @} */
