/*
 * Copyright (C) 2020 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/*
 * @{
 *
 * @file
 * @brief       vl53l0x-api RIOT interface emulation
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

/* Standard include */
#include <assert.h>
#include <stdio.h>
#include <sys/shm.h>

/* Project includes */
#include "vl53l0x.h"
#include "board.h"
#include "platform.h"
#include "shmem.h"

uint16_t *shmem_ptrr = NULL;

int vl53l0x_init_dev(vl53l0x_t dev)
{
    (void) dev;
    return 0;
}

int vl53l0x_reset_dev(vl53l0x_t dev) {
    (void) dev;
    return 0;
}

void vl53l0x_reset(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        assert(vl53l0x_reset_dev(dev) == 0);
    }
}

void vl53l0x_init(void)
{
    for (vl53l0x_t dev = 0; dev < VL53L0X_NUMOF; dev++) {
        assert(vl53l0x_init_dev(dev) == 0);
    }
}

uint16_t vl53l0x_continuous_ranging_get_measure(vl53l0x_t dev)
{
    int shmem_key = shmem_get_key();

    /* Try to initialize shared memory if not already done */
    if(shmem_ptrr == NULL && shmem_key != 0) {
        int shmid = shmget(shmem_key, VL53L0X_NUMOF*sizeof(uint16_t), 0);
        shmem_ptrr = (uint16_t*) shmat(shmid,(void*)0,0);
    }

    /* Return max value if shared memory is not initialized */
    if(shmem_ptrr == NULL) {
        return UINT16_MAX;
    }

    /* printf("Sensor %d = %d\n", dev, shmem_ptrr[dev]); */

    /* Return value from simulator */
    return shmem_ptrr[dev];

}
/** @} */
