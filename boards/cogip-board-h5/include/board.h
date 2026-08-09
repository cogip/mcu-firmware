/*
 * Copyright (C) 2026 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-board-h5 COGIP 2026 (STM32H563)
 * @ingroup     boards_cogip-board-h5
 * @brief       Support for the COGIP 2026 CAN+ETH module (STM32H563RITx)
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#pragma once

#include "cpu.h"
#include "periph_conf.h"

/* The motor_driver / encoder helpers are cogip modules; only pull them when
 * they are actually part of the build (e.g. the motion firmware). This keeps
 * the board usable by plain RIOT test applications that do not use them. */
#ifdef MODULE_MOTOR_DRIVER
#include <motor_driver.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Not connected GPIO for fake use (broken out to the J8/J9 connectors) */
#define GPIO_OUTPUT_UNUSED GPIO_PIN(PORT_B, 8)

/* Motor 0 brake: the H5 J8 pinout routes this brake to PB3 (it is PC8 on the
 * G474 cogip-board). Overrides the platform default in motion_motors_params. */
#define MOTION_MOTOR0_BRAKE_PIN GPIO_PIN(PORT_B, 3)

/* LEDs (BLINK_LED net, U2 pin 56) */
#define HEARTBEAT_LED GPIO_PIN(PORT_B, 4)

/* Encoder mode */
#ifdef MODULE_MOTOR_DRIVER
#define COGIP_BOARD_ENCODER_MODE cogip::encoder::EncoderMode::ENCODER_MODE_X4
#endif

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

/** @} */
