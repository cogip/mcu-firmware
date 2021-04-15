/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    driver_lds01_dma LDS01 LiDAR driver
 * @ingroup     drivers
 * @brief       LDS01 (aka HLS-LFCD2) LiDAR driver
 *
 * The LDS01 (aka HLS-LFCD2) device is a 360° 2D LiDAR distance sensor.
 * See specifications [here](https://emanual.robotis.com/assets/docs/LDS_Basic_Specification.pdf).
 *
 * Distances are reported in millimeters.
 *
 * This driver is not generic, it considers using our boards
 * (based on nucleo-f446re), with LDS01 device connected to USART3,
 * using DMA 1, Stream 1, Channel 4 for peripheral to memory transfers.
 *
 * @{
 * @file
 * @brief       Public API for LDS01 driver
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdint.h>
#include <stdbool.h>

/* RIOT includes */
#include "mutex.h"
#include "ringbuffer.h"

/* Periph includes */
#include "periph/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LDS01_NB_ANGLES (360U)                      /**< number of angles (360°) */

typedef void (*lds01_new_frame_cb_t)(void);         /**< new frame callback signature */

/**
 * @brief   LDS01 parameters
 */
typedef struct {
    uart_t uart;                        /**< UART device */
    lds01_new_frame_cb_t new_frame_cb;  /**< function called when a new frame is available */
    bool invert_data;                   /**< invert data (false by default) */
} lds01_params_t;

/**
 * @brief   Default LDS01 identifier definition
 */
typedef unsigned int lds01_t;

/**
 * @brief Initialize LDS01 device
 *
 * @param[in]   lds01   lds01 device id to initialize
 * @param[in]   params  lds01 parameters
 *
 * @return              0 on success, not 0 otherwise
 */
int lds01_init(const lds01_t lds01, const lds01_params_t *params);

/**
 * @brief Update data arrays
 *
 * Get the last fame and update corresponding indexes in data arrays
 *
 * @param[in]   lds01   lds01 device id
 */
void lds01_update_last_frame(const lds01_t lds01);

/**
 * @brief Start LDS01 device
 *
 * @param[in]   lds01       lds01 device id
 */
void lds01_start(const lds01_t lds01);

/**
 * @brief Stop LDS01 device
 *
 * @param[in]   lds01       lds01 device id
 */
void lds01_stop(const lds01_t lds01);

/**
 * @brief Set new distance filter value
 *
 * @param[in]   lds01       lds01 device id
 * @param[in]   new_filter  value of the new filter, in millimeters, 0 means no filter
 */
void lds01_set_distance_filter(const lds01_t lds01, uint16_t new_filter);

/**
 * @brief Set new minimun intensity value used to validate distances
 *
 * @param[in]   lds01       lds01 device id
 * @param[in]   new_filter  value of the new minimun intensity
 */
void lds01_set_min_intensity(const lds01_t lds01, uint16_t new_min_intensity);

/**
 * @brief Get distance data
 *
 * @param[in]   lds01      lds01 device id
 * @param[out]  distances  array pointer to copy distance data
 */
void lds01_get_distances(const lds01_t lds01, uint16_t *distances);

/**
 * @brief Get intentity data
 *
 * @param[in]   lds01        lds01 device id
 * @param[out]  intensities  array pointer to copy intensity data
 */
void lds01_get_intensities(const lds01_t lds01, uint16_t *intensities);

#ifdef __cplusplus
}
#endif

/** @} */
