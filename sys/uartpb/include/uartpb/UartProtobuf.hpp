// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_uartpb
/// @brief       Exchange Protobuf messages over UART module.
/// @details     This module provides a class that controls an UART to send and
///              receive Protobuf messages using EmbbededProto.
///              A callback function must also be provided to handle decoded messages.
/// @{
/// @file
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// RIOT includes
#include "riot/mutex.hpp"
#include "periph/uart.h"
#include "ringbuffer.h"
#include "thread.h"

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
    using message_handler_t = void (*)(uint8_t message_type, cogip::uartpb::ReadBuffer &);

    /// Class constructor.
    UartProtobuf(
        message_handler_t message_handler,        ///< [in] callback to process the message after decoding
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

    /// Send an empty message (just a message type without data) on serial port.
    void send_message(
        uint8_t message_type                      ///< [in] message type
        );

    /// Send Protobuf message on serial port.
    /// @return true if message was encoded and sent, false otherwise
    bool send_message(
        uint8_t message_type,                     ///< [in] message type
        const EmbeddedProto::MessageInterface &message
                                                  ///< [in] message to send
        );

private:
    uart_t uart_dev_;                             ///< UART device
    uint32_t uart_speed_;                         ///< UART baud rate
    kernel_pid_t reader_pid_;                     ///< reader thread PID
    char reader_stack_[UARTPB_READER_STACKSIZE];  ///< reader thread stack
    char rx_mem_[UART_BUFFER_SIZE];               ///< memory for UART incoming bytes
    ringbuffer_t rx_buf_;                         ///< ring buffer for UART incoming bytes
    uint8_t msg_type_ = UINT8_MAX;                ///< type of message to receive
    uint8_t msg_length_bytes_number_ = 4;         ///< number of bytes needed to encode message length
    uint8_t msg_length_bytes_received_ = 0;       ///< number of bytes received for current message length
    uint32_t msg_length_ = 0;                     ///< message length
    uint32_t msg_bytes_received_ = 0;             ///< number of bytes received for current message
    ReadBuffer read_buffer_;                      ///< buffer used to decode a message
    WriteBuffer write_buffer_;                    ///< buffer used to encode a message
    void (*message_handler_)(uint8_t message_type, cogip::uartpb::ReadBuffer &);
                                                  ///< callback to process the message after decoding
    riot::mutex mutex_;                           ///< mutex protecting serial port access
};

} // namespace uartpb

} // namespace cogip

/// @}
