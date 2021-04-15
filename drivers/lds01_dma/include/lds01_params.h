/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_lds01_dma
 * @{
 *
 * @file
 * @brief       Default parameters for LDS01 driver
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */


#error This default header should not be included. \
    Create your own lds01_params.h in your application folder.

/* Code below is given as example */

#pragma once

#include "lds01.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   LDS01 configuration
 */
static const lds01_params_t lds01_params[] = {
    {
        .uart = UART_DEV(2),
        .new_frame_cb = NULL,
        .invert_data = false
    }
};

#ifdef __cplusplus
}
#endif

/** @} */
