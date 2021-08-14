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
 * @{
 * @file
 * @brief       Private API for Trace file descriptor module
 *
 * This header provides function declarations that have different implementations
 * whether the module is compiled for a native CPU or a real hardware
 *
 * @author      Eric Courtois <eric.courtois@gmail.com>
 */

#pragma once

namespace cogip {

namespace tracefd {

bool init_root_dir(void);

} // namespace tracefd

} // namespace cogip

/** @} */
