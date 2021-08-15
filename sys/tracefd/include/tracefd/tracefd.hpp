// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    sys_tracefd Trace file descriptor module
/// @ingroup     sys
/// @brief       Trace file descriptor module.
///              This module provides an API to print traces on `stderr`, `stdout` or files on sdcard.
///              Since it may be difficult to determine when to close the files
///              to effectively flush data on disk, and to avoid opening/closing files
///              around each printf, this module starts a thread that
///              periodically closes/opens files to flush data.
///              This thread can be disabled to allow manual handling of opening/closing files.
///              Using the lock/unlock API, it is possible to execute
///              consecutive prints in thread safe manner.
///              The module relies on 'sdcard_spi' driver, so SPI parameters must be
///              provided in the board/platform/application.
///              Native boards are also supported, files being written in '/tmp'.
/// @{
/// @file
/// @brief       Public API for Trace file descriptor module
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "tracefd/File.hpp"

/// @def TRACEFD_ROOT_DIR
/// @brief Base directory of trace files
#ifndef TRACEFD_ROOT_DIR
#  define TRACEFD_ROOT_DIR "/tmp/sdcard"
#endif

/// @def TRACEFD_FLUSH_INTERVAL
/// @brief Interval in ms between two flush of all files
#ifndef TRACEFD_FLUSH_INTERVAL
#  define TRACEFD_FLUSH_INTERVAL 1000
#endif

/// @def TRACEFD_THREAD_PRIORITY
/// @brief Files flusher thread priority
#ifndef TRACEFD_THREAD_PRIORITY
#  define TRACEFD_THREAD_PRIORITY (THREAD_PRIORITY_MAIN - 1)
#endif

/// @def TRACEFD_ROOT_DIR
/// @brief Base directory of trace files
#ifndef TRACEFD_ROOT_DIR
#  define TRACEFD_ROOT_DIR "/tmp/sdcard"
#endif

namespace cogip {

namespace tracefd {

extern File out;                /// trace descriptor bound to stdout
extern File err;                /// trace descriptor bound to stderr

void initialize_tracefd(void);  /// Initialize root dir and flusher thread
void flush_all(void);           /// Flush all the trace files
void start_files_flusher(void); /// Start the files flusher thread
void stop_files_flusher(void);  /// Stop the files flusher thread

} // namespace tracefd

} // namespace cogip

/// @}
