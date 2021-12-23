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
 * @brief       LX-Servos messages reader
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 *
 * @}
 */

#include "lx_reader.h"

static uint8_t _compute_sum(const lx_reader_t *reader)
{
    uint8_t sum = 0;

    for (size_t i = 2; i < reader->size - 1; i++) {
        sum += reader->buffer[i];
    }
    return sum;
}

uint8_t lx_reader_compute_sum(const lx_reader_t *reader)
{
    return ~_compute_sum(reader);
}

bool lx_reader_is_valid(const lx_reader_t *reader)
{
    return
        lx_reader_check_minsize(reader) &&
        lx_reader_check_start(reader)   &&
        lx_reader_check_size(reader)    &&
        lx_reader_check_sum(reader);
}
