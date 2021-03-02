/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_shmem Shared memory module
 * @ingroup     sys
 * @brief       Shared memory module
 *
 * Module used to map a shared memory segment initialized externally,
 * for example by a simulator.
 *
 * The structure `shmem_data_t` must be defined in `shmem_params.h`
 * in the application folder.
 *
 * @{
 * @file
 * @brief       Public API for shmem module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* RIOT includes */
#include "shell.h"

#include "shmem_params.h"

#ifdef __cplusplus
extern "C" {
#endif

void shmem_set_key(int key);
int shmem_get_key(void);
const shmem_data_t *shmem_get_data(void);
int shmem_set_key_cmd_cb(int argc, char **argv);

extern const shell_command_t shmem_set_key_cmd;

#ifdef __cplusplus
}
#endif

/** @} */
