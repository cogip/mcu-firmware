/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    driver_lds01 LDS01 (aka HLS-LFCD2) LiDAR driver
 * @ingroup     drivers_sensors
 * @brief       LDS01 (aka HLS-LFCD2) LiDAR driver
 *
 * The LDS01 (aka HLS-LFCD2) device is a 360° 2D LiDAR distance sensor.
 * See specifications [here](https://emanual.robotis.com/assets/docs/LDS_Basic_Specification.pdf).
 *
 * Distances are reported in millimeters.
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

/**
 * @brief   LDS01 offset
 */
typedef struct {
    uint8_t intensity[2];   /**< intentity bytes */
    uint8_t distance[2];    /**< distance bytes */
    uint8_t reserved[2];    /**< reserved bytes */
} lds01_offset_t;

/**
 * @brief   LDS01 frame
 */
typedef struct {
    uint8_t sync;               /**< begin frame byte (0xAF) */
    uint8_t index;              /**< frame index */
    uint8_t rpm[2];             /**< rpm (unused)*/
    lds01_offset_t offsets[6];  /**< frame offsets */
    uint8_t checksum[2];        /**< frame checksum */
} lds01_frame_t;

#define LDS01_UART_BAUD (230400U)                   /**< lds01 UART baud rate */
#define LDS01_FRAME_SIZE (sizeof(lds01_frame_t))    /**< lds01 frame size */
#define LDS01_BUFFER_SIZE (LDS01_FRAME_SIZE * 4)    /**< size of the ringbuffer used to store raw data from UART */
#define LDS01_NB_ANGLES (360U)                      /**< number of angles (360°) */

typedef void (*lds01_new_frame_cb_t)(void);         /**< new frame callback signature */

/**
 * @brief   LDS01 parameters
 */
typedef struct {
    uart_t uart;                        /**< UART device */
    lds01_new_frame_cb_t new_frame_cb;  /**< function called when a new frame is available */
} lds01_params_t;

/**
 * @brief   LDS01 descriptor
 */
typedef struct {
    lds01_params_t params;                  /**< parameters */
    bool running;                           /**< set to true if lds01 is running */
    uint16_t filter;                        /**< distance filter in millimeters */
    mutex_t data_lock;                      /**< lock protecting data access */
    uint16_t distances[LDS01_NB_ANGLES];    /**< distance data */
    uint16_t intensities[LDS01_NB_ANGLES];  /**< intensity data */
    char rx_mem[LDS01_BUFFER_SIZE];         /**< raw frame buffer */
    ringbuffer_t rx_buf;                    /**< frame ringbuffer */
    uint8_t remaining_bytes_in_frame;       /**< number of bytes to read for a full frame */
} lds01_t;

/**
 * @brief Initialize LDS01 device
 *
 * @param[out]  lds01   lds01 device
 * @param[in]   params  lds01 parameters
 *
 * @return              0 on success, not 0 otherwise
 */
int lds01_init(lds01_t *lds01, const lds01_params_t *params);

/**
 * @brief Update data arrays
 *
 * Get the last fame and update corresponding indexes in data arrays
 *
 * @param[in]   lds01       lds01 device
 */
void lds01_update_last_frame(lds01_t *lds01);

/**
 * @brief Start LDS01 device
 *
 * @param[in]   lds01       lds01 device
 */
void lds01_start(lds01_t *lds01);

/**
 * @brief Stop LDS01 device
 *
 * @param[in]   lds01       lds01 device
 */
void lds01_stop(lds01_t *lds01);

/**
 * @brief Set new distance filter value
 *
 * @param[in]   lds01       lds01 device
 * @param[in]   new_filter  value of the new filter, in millimeters, 0 means no filter
 */
void lds01_set_distance_filter(lds01_t *lds01, uint16_t new_filter);

/**
 * @brief Get distance data
 *
 * @param[in]   lds01      lds01 device
 * @param[out]  distances  array pointer to copy distance data
 */
void lds01_get_distances(lds01_t *lds01, uint16_t *distances);

/**
 * @brief Get intentity data
 *
 * @param[in]   lds01        lds01 device
 * @param[out]  intensities  array pointer to copy intensity data
 */
void lds01_get_intensities(lds01_t *lds01, uint16_t *intensities);

#ifdef __cplusplus
}
#endif

/** @} */
