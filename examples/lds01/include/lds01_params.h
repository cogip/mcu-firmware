/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lds01
 * @{
 *
 * @file
 * @brief       LDS01 parameters for current application
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

#include "lds01.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   New frame available callback, defined in main.c
 */
extern void new_frame_available_cb(void);

/**
 * @brief   LDS01 configuration
 */
static const lds01_params_t lds01_params[] = {
    {
        .uart = UART_DEV(2),
        .new_frame_cb = new_frame_available_cb,
        .invert_data = false
    }
};

#ifdef __cplusplus
}
#endif

/** @} */
