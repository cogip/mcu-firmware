/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-board-pami COGIP 2024 PAMI robots board
 * @ingroup     boards_cogip-board-ng
 * @brief       Support for the COGIP 2024 PAMI robots embedded board
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

#include <motor_driver.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Not connected GPIO for fake use */
#define GPIO_OUTPUT_UNUSED     GPIO_PIN(PORT_B, 15)

/* LEDs */
#define HEARTBEAT_LED   GPIO_PIN(PORT_A, 0)

/* GPIOs expander */
#define PCF857X_PORT_0  0

/* Servomotors */
#define LX_DIR_PIN      GPIO_PIN(PORT_B, 3)
#define LX_UART_DEV     2

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

/** @} */
