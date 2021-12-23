/*
 * Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lx_servos
 * @{
 *
 * @file
 * @brief       LX-Servos messages writer
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 *
 * @}
 */

#include "lx_writer.h"

void lx_writer_init(lx_writer_t *writer, uint8_t *buffer, size_t buffer_max_size)
{
    writer->buffer = buffer;
    writer->size = 0;
    writer->buffer_max_size = buffer_max_size;
}

const uint8_t *lx_writer_get_data(const lx_writer_t *writer)
{
    return (const uint8_t *)writer->buffer;
}

size_t lx_writer_get_size(const lx_writer_t *writer)
{
    return writer->size;
}

void lx_writer_write_make(lx_writer_t *writer, uint8_t id, lx_write_command_t write_command, const uint8_t *buffer, size_t size)
{
    const size_t len = 3 + size;

    if (len + 3 <= writer->buffer_max_size) {
        writer->size = len + 3;

        uint8_t sum = 0;

        writer->buffer[0] = LX_START;
        writer->buffer[1] = LX_START;
        sum += writer->buffer[2] = id;
        sum += writer->buffer[3] = len;
        sum += writer->buffer[4] = write_command;

        for (size_t i = 0; i < size; i++) {
            sum += writer->buffer[5 + i] = buffer[i];
        }

        writer->buffer[writer->size - 1] = ~sum;
    }
    else {
        writer->size = 0;
    }
}

void lx_writer_read_make(lx_writer_t *writer, uint8_t id, lx_read_command_t read_command)
{
    const size_t len = 3;

    if (len + 3 <= writer->buffer_max_size) {
        writer->size = len + 3;

        uint8_t sum = 0;

        writer->buffer[0] = LX_START;
        writer->buffer[1] = LX_START;
        sum += writer->buffer[2] = id;
        sum += writer->buffer[3] = len;
        sum += writer->buffer[4] = read_command;
        writer->buffer[5] = ~sum;
    }
    else {
        writer->size = 0;
    }
}
