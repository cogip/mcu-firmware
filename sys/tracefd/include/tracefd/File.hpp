// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_tracefd
/// @{
/// @file
/// @brief       File class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Standard includes
#include <string>

// RIOT includes
#include "riot/mutex.hpp"

namespace cogip {

namespace tracefd {

class File {
public:
    /// Constructor based on file name.
    /// Create a new trace file descriptor, also starts the files flusher thread.
    /// @param[in]   filename   name of the trace file
    File(const std::string & filename);

    /// Constructor based on file descriptor.
    /// Only used to initialize `out` and `err` descriptors.
    /// @param[in]   f                 file descriptor
    File(FILE *f);

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

} // namespace tracefd

} // namespace cogip

/// @}
