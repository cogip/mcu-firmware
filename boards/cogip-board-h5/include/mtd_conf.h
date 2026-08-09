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
 * @brief       MTD configuration for persistent settings storage (STM32H563)
 *
 * Reserves the last flash sectors of the STM32H563RI (2 MB, dual-bank,
 * homogeneous 8 KiB sectors) for the FlashDB key-value settings region.
 *
 * As on the G474 board we cannot use MTD_FLASHPAGE_INIT_VAL() because it
 * exposes the whole flash (256 sectors) to FlashDB, which would put the
 * firmware code region at risk. Instead the device is limited to the last
 * 4 sectors (32 KiB).
 *
 * Memory layout (last 4 sectors of flash):
 * - Offset:        FLASHPAGE_NUMOF - 4 = sector 252 (on STM32H563RI)
 * - Sector count:  4
 * - Pages/sector:  1
 * - Page size:     8192 bytes (FLASHPAGE_SIZE, hardware sector size)
 * - Total size:    4 x 8192 = 32 KiB
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef MTD_SETTINGS_H
#define MTD_SETTINGS_H

#include "mtd_flashpage.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MTD_SETTINGS_SECTOR_COUNT (4)
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

#ifdef __cplusplus
}
#endif

#endif /* MTD_SETTINGS_H */
/** @} */
