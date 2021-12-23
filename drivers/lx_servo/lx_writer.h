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
 * @brief       Interface definition for LX-Servos packet writer
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef LX_WRITER_H
#define LX_WRITER_H

#include "lx_protocol.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LX-Servos packet writer struct
 */
typedef struct {
    uint8_t *buffer;        /**< data buffer */
    size_t size;            /**< packet's size */
    size_t buffer_max_size; /**< data buffer's size */
} lx_writer_t;

/**
 * @brief Initialize the LX-Servos packet writer
 *
 * @param[out] writer    the packet writer
 * @param[in] buffer     the buffer used to store data
 * @param[in] buffer_max_size      the size of the buffer (= maximum packet size)
 */
void lx_writer_init(lx_writer_t *writer, uint8_t *buffer, size_t buffer_max_size);

/**
 * @brief Get the data buffer to send
 *
 * @param[out] writer    the packet writer
 *
 * @return the beginning address of the buffer
 */
const uint8_t *lx_writer_get_data(const lx_writer_t *writer);

/**
 * @brief Get the data buffer's size to send
 *
 * @param[out] writer    the packet writer
 *
 * @return the buffer's size
 */
size_t lx_writer_get_size(const lx_writer_t *writer);

/**
 * @brief Build a WRITE packet
 *
 * @param[out] writer    the packet writer
 * @param[in] id         the destination's id
 * @param[in] reg        the register to write in
 * @param[in] buffer     the data buffer to write
 * @param[in] size       the data buffer's size
 */
void lx_writer_write_make(lx_writer_t *writer, uint8_t id, uint8_t reg, const uint8_t *buffer, size_t size);

/**
 * @brief Build a READ packet
 *
 * @param[out] writer       the packet writer
 * @param[in] id            the destination's id
 * @param[in] read_command  the parameter read command
 */
void lx_writer_read_make(lx_writer_t *writer, uint8_t id, lx_read_command_t read_command);

#ifdef __cplusplus
}
#endif

#endif /* LX_WRITER_H */
/** @} */
