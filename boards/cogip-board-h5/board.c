/*
 * Copyright (C) 2026 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip-board-h5
 * @{
 *
 * @file
 * @brief       Board initialization code for cogip-board-h5
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 *
 * @}
 */

#include "board.h"
#include <stdio.h>

#include "mtd_conf.h"
#include "periph/gpio.h"

/*
 * Settings storage MTD (FlashDB backend), backed by the last 4 flash sectors
 * (32 KiB) via the STM32H5 flashpage driver. Registered as MTD_0
 * (mtd_dev_get(0), i.e. FAL_MTD), giving MTD_NUMOF == 1.
 */
static mtd_flashpage_t mtd_settings = MTD_SETTINGS_INIT_VAL;
MTD_XFA_ADD(mtd_settings, 0);

/**
 * Start of the heap
 */
extern char _sheap;
/**
 * End of the heap
 */
extern char _eheap;

/**
 * Pointer to start of the heap
 */
char* __sheap = &_sheap;
/**
 * Pointer to end of the heap
 */
char* __eheap = &_eheap;

/**
 * Board init
 */
void board_init(void)
{
    // Setup and set heartbeat LED
    gpio_init(HEARTBEAT_LED, GPIO_OUT);
    gpio_set(HEARTBEAT_LED);

    puts("Board successfully initialized.");
}
