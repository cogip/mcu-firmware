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

// Standard includes
#include <string>
#include <vector>

// RIOT includes
#include "riot/mutex.hpp"

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

class file {
public:
    /// Constructor based on file name.
    /// Create a new trace file descriptor, also starts the files flusher thread.
    /// @param[in]   filename   name of the trace file
    file(const std::string & filename);

    /// Constructor based on file descriptor.
    /// Only used to initialize `out` and `err` descriptors.
    /// @param[in]   f                 file descriptor
    file(FILE *f);

    /// Open the trace file
    void open(void);

    /// Close the trace file
    void close(void);

    /// Lock the trace file
    void lock(void);

    /// Unlock the trace file
    void unlock(void);

    /// Print formatted string in the trace file.
    /// File descriptor must be locked/unlocked manually before/after
    /// calling this function.
    /// @param[in]   format     format string
    /// @param[in]   ...        arguments
    void printf(const char *format, ...);

    /// Print formatted string in the trace file in a JSON log message.
    /// The message must not contain '\n' nor '"'.
    /// File descriptor is automatically locked/unlocked around print.
    /// @param[in]   format     format string
    /// @param[in]   ...        arguments
    void logf(const char *format, ...);

    /// Flush the trace file (close and re-open).
    void flush(void);

private:
    FILE * file_;               /// file descriptor
    std::string filename_;      /// filename
    riot::mutex mutex_;         /// mutex protecting file access
};

extern file out;                /// trace descriptor bound to stdout
extern file err;                /// trace descriptor bound to stderr

void flush_all(void);           /// Flush all the trace files
void start_files_flusher(void); /// Start the files flusher thread
void stop_files_flusher(void);  /// Stop the files flusher thread

} // namespace tracefd

} // namespace cogip

/** @} */
