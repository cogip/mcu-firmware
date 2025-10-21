/*
 * Copyright (C) 2025 COGIP
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_cogip_native
 * @{
 *
 * @file
 * @brief       MTD configuration for persistent settings storage
 *
 * This file defines the MTD (Memory Technology Device) configuration
 * for persistent key-value storage using FlashDB on the cogip-native board.
 *
 * @author      Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 */

#ifndef MTD_SETTINGS_H
#define MTD_SETTINGS_H

#include "mtd.h"
#include "mtd_emulated.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MTD Memory Technology Device interface configuration for settings storage
 *
 * Configuration for persistent key-value storage using FlashDB.
 * Uses emulated MTD in RAM for native board (x86_64 simulation).
 *
 * Memory layout:
 * - Number of sectors: 8
 * - Pages per sector: 1
 * - Page size: 2048 bytes
 * - Total size: 16 KB
 *
 * @{
 */
#define MTD_SETTINGS_PAGE_NUMBER (8)
#define MTD_SETTINGS_INIT_VAL MTD_EMULATED_DEV(0, MTD_SETTINGS_PAGE_NUMBER, 1, FAL_PART0_LENGTH)

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MTD_SETTINGS_H */
/** @} */
