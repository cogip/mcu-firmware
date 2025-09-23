// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    sys_canpb CAN Protobuf module
/// @ingroup     sys
/// @brief       Exchange Protobuf messages over CAN module.
/// @{
/// @file
/// @author      Gilles DOFFE <g.doffe@gmail.com>
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#ifndef CANPB_READER_PRIO
#define CANPB_READER_PRIO (THREAD_PRIORITY_MAIN - 1) ///< message reader thread priority
#endif

#ifndef CANPB_READER_STACKSIZE
#define CANPB_READER_STACKSIZE THREAD_STACKSIZE_MAIN ///< message reader thread stask size
#endif

#ifndef CANPB_INPUT_MESSAGE_LENGTH_MAX
#define CANPB_INPUT_MESSAGE_LENGTH_MAX 1024 ///< max incoming message length
#endif

#ifndef CANPB_OUTPUT_MESSAGE_LENGTH_MAX
#define CANPB_OUTPUT_MESSAGE_LENGTH_MAX (4 * 1024) ///< max outgoing message length
#endif

#ifndef CANPB_MAX_HANDLERS
#define CANPB_MAX_HANDLERS 16 ///< max numbers of registered message handlers
#endif

/// @}
