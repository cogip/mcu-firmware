/*
 * Copyright (C) 2021 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_cogip-nucleo-f446re COGIP test board
 * @ingroup     boards
 * @brief       Support for COGIP test board, inherited from nucleo-f446re
 * @{
 *
 * @file
 * @brief       Pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

#include "board_nucleo.h"
#include "arduino_pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs */
#define HEARTBEAT_LED   GPIO_PIN(PORT_A, 5)

/* Motors */
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

/* Quadrature decoding */
#define QDEC_MODE           QDEC_X4
#define QDEC_LEFT_POLARITY  -1
#define QDEC_RIGHT_POLARITY -1

#ifdef __cplusplus
}
#endif
