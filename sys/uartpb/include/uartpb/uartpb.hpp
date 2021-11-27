// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    sys_uartpb UART Protobuf module
/// @ingroup     sys
/// @brief       Exchange Protobuf messages over UART module.
/// @{
/// @file
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#ifndef UARTPB_READER_PRIO
#define UARTPB_READER_PRIO       (THREAD_PRIORITY_MAIN - 1)  ///< message reader thread priority
#endif

#ifndef UARTPB_READER_STACKSIZE
#define UARTPB_READER_STACKSIZE  THREAD_STACKSIZE_MAIN       ///< message reader thread stask size
#endif

#ifndef UARTPB_INPUT_MESSAGE_LENGTH_MAX
#define UARTPB_INPUT_MESSAGE_LENGTH_MAX  128                 ///< max incoming message length
#endif

#ifndef UARTPB_OUTPUT_MESSAGE_LENGTH_MAX
#define UARTPB_OUTPUT_MESSAGE_LENGTH_MAX  512                ///< max outgoing message length
#endif

/// @}
