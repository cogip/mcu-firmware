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
#include "platform.h"

/* SHM structure representing the memory shared with the simulator */
typedef struct {
    uint16_t vl53l0x_values[VL53L0X_NUMOF];
} shmem_data_t;

/** @} */
