/*
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_native
 *
 * The native board uses call level hardware simulation
 *
 * @{
 *
 * @file
 * @brief       Basic definitions for the native board
 *
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 */

#pragma once

#include <stdint.h>

/* RIOT includes */
#include <motor_driver.h>

/* Project includes */
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fake GPIOs */
enum { PORT_A = 0, PORT_B, PORT_C, PORT_D, PORT_E, NATIVE_GPIO_PORT_NUMOF };

/* Heartbeat LED */
#define HEARTBEAT_LED GPIO_PIN(PORT_A, 0)

/* Servomotors */
#define LX_UART_DEV 2

/* Encoder mode */
/* QDEC on native architecture only support X1 mode, so force it here */
#define COGIP_BOARD_ENCODER_MODE cogip::encoder::EncoderMode::ENCODER_MODE_X1

/* Motion motors simulation */
#define MOTION_MOTORS_POST_CB cogip_native_motor_driver_qdec_simulation

/**
 * @name    ztimer configuration
 * @{
 */
#define CONFIG_ZTIMER_USEC_TYPE ZTIMER_TYPE_PERIPH_TIMER
#define CONFIG_ZTIMER_USEC_DEV TIMER_DEV(0)
/* on native, anything can happen... */
#define CONFIG_ZTIMER_USEC_MIN (64)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
