/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lx_servos
 *
 * @{
 *
 * @file
 * @brief       Interface definition for LX-Servos packet reader
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef LX_READER_H
#define LX_READER_H

#include "lx_protocol.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LX-Servos packet reader struct
 */
typedef struct {
    const uint8_t *buffer;  /**< data buffer */
    size_t size;            /**< data buffer's size */
} lx_reader_t;

/**
 * @brief Initialize the LX-Servos packet reader
 *
 * @param[out] reader    the packet reader
 * @param[in] buffer     the buffer used to store data
 * @param[in] size       the size of the buffer
 */
static inline void lx_reader_init(lx_reader_t *reader, const uint8_t *buffer, size_t size)
{
    reader->buffer = buffer;
    reader->size = size;
}

/**
 * @brief Compute the packet's sum
 *
 * @param[in] reader      the packet reader
 *
 * @return the sum of the packet
 */
uint8_t lx_reader_compute_sum(const lx_reader_t *reader);

/**
 * @brief Check if the packet has the minimum required size
 *
 * @param[in] reader      the packet reader
 *
 * @return true if the packet has the minimum required size
 * @return false otherwise
 */
static inline bool lx_reader_check_minsize(const lx_reader_t *reader)
{
    return LX_PACKET_SIZE(0) < reader->size;
}

/**
 * @brief Check if the packet begins with 2 LX_START bits
 *
 * @param[in] reader      the packet reader
 *
 * @return true if the packet begins with 2 LX_START bits
 * @return false otherwise
 */
static inline bool lx_reader_check_start(const lx_reader_t *reader)
{
    return
        reader->buffer[0] == LX_START &&
        reader->buffer[1] == LX_START;
}

/**
 * @brief Check if the packet's size is the same as the buffer's size
 *
 * @param[in] reader      the packet reader
 *
 * @return true if the packet's size is the same as the buffer's size
 * @return false otherwise
 */
static inline bool lx_reader_check_size(const lx_reader_t *reader)
{
    return reader->size == (size_t)(reader->buffer[3] + LX_PACKET_HEADER_SIZE);
}

/**
 * @brief Check if the computed sum and the sum of the packet are equal
 *
 * @param[in] reader      the packet reader
 *
 * @return true if the computed sum and the sum of the packet are equal
 * @return false otherwise
 */
static inline bool lx_reader_check_sum(const lx_reader_t *reader)
{
    return lx_reader_compute_sum(reader) == reader->buffer[reader->size - 1];
}

/**
 * @brief Check if the packet is valid
 *
 * @param[in] reader      the packet reader
 *
 * @return true if the packet is valid
 * @return false otherwise
 */
bool lx_reader_is_valid(const lx_reader_t *reader);

/**
 * @brief Get the packet's device id
 *
 * @param[in] reader      the packet reader
 *
 * @return the packet's device id
 */
static inline uint8_t lx_reader_get_id(const lx_reader_t *reader)
{
    return reader->buffer[2];
}

/**
 * @brief Get the packet's instruction code
 *
 * @param[in] reader      the packet reader
 *
 * @return the packet's instruction code
 */
static inline uint8_t lx_reader_get_instr(const lx_reader_t *reader)
{
    return reader->buffer[4];
}

/**
 * @brief Get the packet's payload (response)
 *
 * @param[in] reader      the packet reader
 *
 * @return the address of the beginning of the payload
 */
static inline const uint8_t *lx_reader_response_get_payload(const lx_reader_t *reader)
{
    return &reader->buffer[5];
}

/**
 * @brief Get the packet's payload size (response)
 *
 * @param[in] reader      the packet reader
 *
 * @return the size of the payload
 */
static inline size_t lx_reader_response_get_payload_size(const lx_reader_t *reader)
{
    return reader->buffer[3];
}

/**
 * @brief Get the packet's READ size
 *
 * @param[in] reader      the packet reader
 *
 * @return the READ size
 */
static inline size_t lx_reader_read_get_size(const lx_reader_t *reader)
{
    return reader->buffer[6];
}

#ifdef __cplusplus
}
#endif

#endif /* LX_READER_H */
/** @} */
