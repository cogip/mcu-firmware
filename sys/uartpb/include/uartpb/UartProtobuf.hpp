// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_uartpb
/// @brief       Exchange Protobuf messages over UART module.
/// @details     This module provides a class that controls an UART to send and
///              receive Protobuf messages using EmbbededProto.
///              Each library/application using uartpb must define a unique id (32-bit integer
///              of type cogip::uartpb::uuid_t) and registrer a callback function to send
///              or receive their own messages.
///              See `examples/uartb`.
/// @{
/// @file
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// RIOT includes
#include <mutex.h>
#include "periph/uart.h"
#include "ringbuffer.h"
#include "thread.h"
#include "etl/map.h"

#include "uartpb/uartpb.hpp"
#include "uartpb/ReadBuffer.hpp"
#include "uartpb/WriteBuffer.hpp"

#include "MessageInterface.h"

// Double the size of the ringbuffer receiving input bytes,
// so we can continue receiving data even if the thread decoding
// the message has not consumed previous message immediately.
#define UART_BUFFER_SIZE UARTPB_INPUT_MESSAGE_LENGTH_MAX*2

namespace cogip {

namespace uartpb {

/// Custom type for uuids
using uuid_t = uint32_t;

/// Prototype for incoming Protobuf message handlers
using message_handler_t = std::function<void(cogip::uartpb::ReadBuffer *)>;

/// Function called when data is received on serial port.
/// This wrapper is used to call the uart_rx_cb() function from UartProtobuf class
/// from a C context, passing the UartProtobuf instance pointer as first parameter.
void uart_rx_cb_wrapper(
    void *arg,                                    ///< [in] pointer to UartProtobuf instance
    uint8_t data                                  ///< [in] incoming data
    );

/// Thread function decoding incoming Protobuf messages.
/// This wrapper is used to call the message_reader() function from UartProtobuf class
/// from C context, passing the UartProtobuf instance pointer as first parameter.
void *message_reader_wrapper(
    void *arg                                     ///< [in] pointer to UartProtobuf instance
    );

/// Generic UART Protobuf communication class.
class UartProtobuf {
public:

    /// Class constructor.
    UartProtobuf(
        uart_t uart_dev,                          ///< [in] UART device
        uint32_t uart_speed=230400U               ///< [in] UART baud rate
        );

    /// Open serial port.
    /// @return true if serial port is opened, false otherwise
    bool connect();

    /// Start thread waiting for incoming messages.
    void start_reader();

    /// Function call for each incomming bytes on serial port.
    void uart_rx_cb(
        uint8_t data                              ///< [in] incoming data
        );

    /// Wait and decode incoming Protobuf messages.
    void message_reader();

    /// Send message on serial port.
    /// @return true if message was encoded and sent, false otherwise
    bool send_message(
        uuid_t uuid,                              ///< [in] message uuid
        const EmbeddedProto::MessageInterface *message = nullptr
                                                  ///< [in] message to send
        );

    /// Associate a message handle to a specific uuid
    void register_message_handler(
        uuid_t uuid,                              ///< [in] message uuid
        message_handler_t handler                 ///< [in] message handler
        );

private:
    uart_t uart_dev_;                             ///< UART device
    uint32_t uart_speed_;                         ///< UART baud rate
    kernel_pid_t reader_pid_;                     ///< reader thread PID
    ringbuffer_t rx_buf_;                         ///< ring buffer for UART incoming bytes
    uint32_t msg_length_ = 0;                     ///< message length
    mutex_t mutex_ = MUTEX_INIT;                  ///< mutex protecting serial port access
    etl::map<uuid_t, message_handler_t, UARTPB_MAX_HANDLERS> message_handlers_;
                                                  ///< callbacks to process the message after decoding
    char reader_stack_[UARTPB_READER_STACKSIZE];  ///< reader thread stack
    char rx_mem_[UART_BUFFER_SIZE];               ///< memory for UART incoming bytes
    ReadBuffer read_buffer_;                      ///< buffer used to decode a message
    WriteBuffer write_buffer_;                    ///< buffer used to encode a message
};

} // namespace uartpb

} // namespace cogip

/// @}
