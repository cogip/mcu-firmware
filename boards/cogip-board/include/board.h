/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-board COGIP 20234
 * @ingroup     boards_cogip-board
 * @brief       Support for the COGIP 2024
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
#define GPIO_OUTPUT_UNUSED GPIO_PIN(PORT_B, 15)

/* LEDs */
#define HEARTBEAT_LED GPIO_PIN(PORT_A, 5)

/* Encoder mode */
#define COGIP_BOARD_ENCODER_MODE cogip::encoder::EncoderMode::ENCODER_MODE_X4

/* GPIOs expander */
#define PCF857X_PORT_0 0

/* Servomotors */
#define LX_UART_DEV 1

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

/** @} */
