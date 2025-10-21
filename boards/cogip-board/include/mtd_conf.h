/*
 * Copyright (C) 2025 COGIP
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
 * Memory layout:
 * - Number of pages: 8
 * - Page size: 2048 bytes (automatically determined by hardware)
 * - Total size: 16 KB
 *
 * @{
 */
#define MTD_SETTINGS_PAGE_NUMBER (8)
#define MTD_SETTINGS_INIT_VAL MTD_FLASHPAGE_INIT_VAL(MTD_SETTINGS_PAGE_NUMBER)

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MTD_SETTINGS_H */
/** @} */
