/*
 * Copyright (C) 2026 COGIP
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip_board
 * @{
 *
 * @file
 * @brief       MTD configuration for persistent settings storage
 *
 * This file defines the MTD (Memory Technology Device) configuration
 * for persistent key-value storage using FlashDB on the cogip-board.
 *
 * @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 */

#ifndef MTD_SETTINGS_H
#define MTD_SETTINGS_H

#include "mtd_flashpage.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MTD Memory Technology Device interface configuration for settings storage
 *
 * Configuration for persistent key-value storage using FlashDB.
 * Uses the last pages of internal flash memory (STM32G474RE).
 *
 * We cannot use MTD_FLASHPAGE_INIT_VAL() because it sets sector_count
 * to FLASHPAGE_NUMOF (= entire flash, 256 sectors / 512 KB), which would
 * expose the firmware code region to FlashDB. Instead we manually limit
 * the device to 8 pages (16 KB).
 *
 * Memory layout (last 8 pages of flash):
 * - Offset: FLASHPAGE_NUMOF - 8 = page 248 (on STM32G474RE)
 * - Sector count: 8
 * - Pages per sector: 1
 * - Page size: 2048 bytes (hardware flash page size)
 * - Total size: 8 × 2048 = 16 KB
 *
 * @{
 */
#define MTD_SETTINGS_SECTOR_COUNT (8)
#define MTD_SETTINGS_PAGES_PER_SECTOR (1)
#define MTD_SETTINGS_PAGE_SIZE (FLASHPAGE_SIZE)
#define MTD_SETTINGS_OFFSET (FLASHPAGE_NUMOF - MTD_SETTINGS_SECTOR_COUNT)
#define MTD_SETTINGS_INIT_VAL                                                                      \
    {                                                                                              \
        .base =                                                                                    \
            {                                                                                      \
                .driver = &mtd_flashpage_driver,                                                   \
                .sector_count = MTD_SETTINGS_SECTOR_COUNT,                                         \
                .pages_per_sector = MTD_SETTINGS_PAGES_PER_SECTOR,                                 \
                .page_size = MTD_SETTINGS_PAGE_SIZE,                                               \
                .write_size = 1,                                                                   \
            },                                                                                     \
        .offset = MTD_SETTINGS_OFFSET,                                                             \
    }

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MTD_SETTINGS_H */
/** @} */
