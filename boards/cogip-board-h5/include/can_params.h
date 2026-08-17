/*
 * Copyright (C) 2026 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip-board-h5
 * @{
 *
 * @file
 * @brief       FDCAN controller configuration for the cogip-board-h5 board
 *
 * Shadows the cpu-level cpu/stm32/include/can_params.h (the board include
 * dir precedes the cpu include dir): on this board the TCAN332 transceiver
 * is wired to FDCAN1 on PA11 (RX, CAN_RXD) / PA12 (TX, CAN_TXD), whereas the
 * STM32H5 cpu default targets the H573I-DK layout (FDCAN2 on PB5/PB6, which
 * here would collide with the left encoder on PB6).
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @}
 */

#pragma once

#include "can/device.h"
#include "periph/can.h"

#ifdef __cplusplus
extern "C" {
#endif

/** cogip-board-h5 FDCAN device config */
static const can_conf_t candev_conf[] = {
    {
        .can = FDCAN1,
        .rcc_mask = RCC_APB1HENR_FDCANEN,
        .rx_pin = GPIO_PIN(PORT_A, 11),
        .tx_pin = GPIO_PIN(PORT_A, 12),
        .af = GPIO_AF9,
        .it0_irqn = FDCAN1_IT0_IRQn,
        .it1_irqn = FDCAN1_IT1_IRQn,
        .en_deep_sleep_wake_up = true,
        .ttcm = 0,
        .abom = 1,
        .awum = 1,
        .nart = 0,
        .rflm = 0,
        .txfp = 0,
    },
};

/** cogip-board-h5 FDCAN device parameters */
static const candev_params_t candev_params[] = {
    {
        .name = "can_stm32_0",
    },
};

#ifdef __cplusplus
}
#endif

/** @} */
