/*
 * Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_tracefd Trace file descriptor module
 * @ingroup     sys
 * @brief       Trace file descriptor module
 *
 * This module provides an API to write trace files on sdcard.
 *
 * Since it may be difficult to determine when to close the files
 * to effectively flush data on disk, and to avoid opening/closing files
 * around each printf, this module starts a thread that
 * periodically closes/opens files to flush data.
 * This thread can be disabled to allow manual handling of opening/closing files.
 *
 * Using the lock/unlock API, it is possible to execute
 * consecutive prints in thread safe manner.
 *
 * The module relies on 'sdcard_spi' driver, so SPI parameters must be
 * provided in the board/platform/application.
 *
 * Native boards are also supported, files being written in '/tmp'.
 *
 * @{
 * @file
 * @brief       Public API for Trace file descriptor module
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

/* Standard includes */
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def TRACEFD_NUMOF
 * @brief Number of trace file descriptors
 */
#ifndef TRACEFD_NUMOF
#  define TRACEFD_NUMOF 1
#endif

/**
 * @def TRACEFD_MAX_PATH
 * @brief Maximum filename length
 */
#ifndef TRACEFD_MAX_PATH
#  define TRACEFD_MAX_PATH 256
#endif

/**
 * @def TRACEFD_FLUSH_INTERVAL
 * @brief Interval in ms between two flush of all files
 */
#ifndef TRACEFD_FLUSH_INTERVAL
#  define TRACEFD_FLUSH_INTERVAL 1000
#endif

/**
 * @def TRACEFD_THREAD_PRIORITY
 * @brief Files flusher thread priority
 */
#ifndef TRACEFD_THREAD_PRIORITY
#  define TRACEFD_THREAD_PRIORITY THREAD_PRIORITY_MIN
#endif

/**
 * @def TRACEFD_ROOT_DIR
 * @brief Base directory of trace files
 */
#ifndef TRACEFD_ROOT_DIR
#  define TRACEFD_ROOT_DIR "/tmp/sdcard"
#endif

/**
 * @brief   Trace file descriptor definition
 */
typedef unsigned int tracefd_t;

/**
 * @brief Initialize trace file descriptor context,
 *        also starts the files flusher thread
 * @param[in]   filename          name of the trace file
 * @return                        trace file descriptor
 */
tracefd_t tracefd_init(const char *filename);

/**
 * @brief Open the trace file
 * @param[in]   tracefd  trace file descriptor
 */
void tracefd_open(const tracefd_t tracefd);

/**
 * @brief Close the trace file
 * @param[in]   tracefd  trace file descriptor
 */
void tracefd_close(const tracefd_t tracefd);

/**
 * @brief Lock the trace file
 * @param[in]   tracefd  trace file descriptor
 */
void tracefd_lock(const tracefd_t tracefd);

/**
 * @brief Unlock the trace file
 * @param[in]   tracefd  trace file descriptor
 */
void tracefd_unlock(const tracefd_t tracefd);

/**
 * @brief Print formatted string in the trace file
 * @param[in]   tracefd  trace file descriptor
 * @param[in]   format   format string
 * @param[in]   ...      arguments
 */
void tracefd_printf(const tracefd_t tracefd, const char *format, ...);

/**
 * @brief Flush the trace file (close and re-open)
 * @param[in]   tracefd  trace file descriptor
 */
void tracefd_flush(const tracefd_t tracefd);

/**
 * @brief Flush all the trace files
 */
void tracefd_flush_all(void);

/**
 * @brief Start the files flusher thread
 */
void tracefd_start_files_flusher(void);

/**
 * @brief Stop the files flusher thread
 */
void tracefd_stop_files_flusher(void);

#ifdef __cplusplus
}
#endif

/** @} */
