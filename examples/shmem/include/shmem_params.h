/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shmem
 * @{
 *
 * @file
 * @brief       Parameters for shmem
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

#include <stdint.h>

#define NB_DATA 8

/* SHM structure representing the memory shared with the simulator */
typedef struct {
    uint8_t even[NB_DATA];
    uint16_t odd[NB_DATA];
} shmem_data_t;

/** @} */
