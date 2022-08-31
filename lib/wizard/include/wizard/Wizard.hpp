// Copyright (C) 2022 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    lib_wizard Wizard module
/// @ingroup     lib
/// @brief       Wizard module
///              This module is used to communicate with the Copilot at the beginning of a game
///              to configure dynamic parameters or play a check-list.
///              It provides predefined Protobuf messages (bool, int, float, str, list, ...)
///              that can be sent using uartpb module.
///              After sending a message, the module waits for a response of the same type, with
///              its value updated.
///              A file `uartpb_config.hpp` must be provided by the application, either at lib,
///              platform or application level. This file must define the `PB_OutputMessage` type.
///              This type refers to a Protobuf message that contains a `oneof` attribute
///              defining a `wizard` field.
/// @{
/// @file
/// @brief       Public API for wizard module
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// Riot includes
#include "event.h"

// Project includes
#include "uartpb/UartProtobuf.hpp"

// Protobuf message
#include "PB_Wizard.hpp"

#ifndef WIZARD_NAME_MAX_LENGTH
#  define WIZARD_NAME_MAX_LENGTH 256  ///< Maximum length of the message strings
#endif
#ifndef WIZARD_REP_MAX
#  define WIZARD_REP_MAX 8            ///< Maximum count of repeated fields
#endif

namespace cogip {

namespace wizard {

/// Wizard class
class Wizard {
public:
    /// Protobuf message type. Shortcut for original template type.
    using PB_Message = PB_Wizard<
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_REP_MAX,
        WIZARD_REP_MAX,
        WIZARD_REP_MAX,
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_REP_MAX,
        WIZARD_REP_MAX,
        WIZARD_REP_MAX,
        WIZARD_REP_MAX,
        WIZARD_REP_MAX,
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_REP_MAX,
        WIZARD_NAME_MAX_LENGTH,
        WIZARD_NAME_MAX_LENGTH
        >;

    /// Constructor.
    explicit Wizard(
        cogip::uartpb::UartProtobuf & uartpb  ///< [in] uartpb pointer used to send/receive messages
        );

    /// Message handler for responses.
    void handle_response(
        cogip::uartpb::ReadBuffer & buffer   ///< [in] buffer containing the received message
        );

    /// Send a request and wait for the response.
    /// @return the response
    const PB_Message &request(
        const PB_Message &request            ///< [in] request to send
        );

private:
    /// custom event type including the response
    typedef struct {
        event_t super;
        PB_Message pb_message;
    } wizard_event_t;

    cogip::uartpb::UartProtobuf & uartpb_;   ///< uartpb pointer used to send/receive messages
    wizard_event_t event_;                   ///< event containing the response
    event_queue_t queue_;                    ///< queue receiving response events
    bool queue_claimed_ = false;             ///< whether event queue has been claimed on the thread or not
};

} // namespace wizard

} // namespace cogip

/// @}
