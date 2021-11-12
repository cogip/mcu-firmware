// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @defgroup    sys_uartpb UART Protobuf module
/// @ingroup     sys
/// @brief       Exchange Protobuf messages over UART module.
/// @details     This module provides a generic class that can be instantiated using
///              types and defines generated from Protobuf messages.
///              Two Protobuf messages are required, one for incoming messages, one for
///              outgoing messages.
///              These messages can include 'oneof' attributes (similar to C unions) to
///              wrap several message types inside the main message type.
///              A callback function must also be provided to handle decoded messages.
/// @{
/// @file
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// RIOT includes
#include "periph/uart.h"
#include "ringbuffer.h"
#include "riot/chrono.hpp"
#include "riot/thread.hpp"
#include "thread.h"
#include "xtimer.h"

#include <pb_encode.h>
#include <pb_decode.h>

#ifndef UARTPB_READER_PRIO
#define UARTPB_READER_PRIO       (THREAD_PRIORITY_MAIN - 1)  ///< Message reader thread priority
#endif

#ifndef UARTPB_READER_STACKSIZE
#define UARTPB_READER_STACKSIZE  THREAD_STACKSIZE_MAIN       ///< Message reader thread stask size
#endif

#ifndef INPUT_MESSAGE_LENGTH_MAX
#define INPUT_MESSAGE_LENGTH_MAX  128                        ///< Max incoming message length
#endif

#ifndef OUTPUT_MESSAGE_LENGTH_MAX
#define OUTPUT_MESSAGE_LENGTH_MAX  128                       ///< Max outgoing message length
#endif

namespace cogip {

namespace uartpb {

/// Function called when data is received on serial port.
void uart_rx_cb_wrapper(
    void *arg,                                      ///< [in] pointer to UartProtobuf instance
    uint8_t data                                    ///< [in] incoming data
    );

/// Thread function decoding incoming Protobuf messages.
void *message_reader_wrapper(
    void *arg                                       ///< [in] pointer to UartProtobuf instance
    );

/// Base class defining non-templated functions.
class UartProtobufBase {
public:
    /// Function call for each incomming bytes on serial port.
    virtual void uart_rx_cb(
        uint8_t data                                ///< [in] incoming data
    ) = 0;

    /// Wait and decode incoming Protobuf messages.
    virtual void message_reader() = 0;
};

/// Generic UART Protobuf communication class.
/// @tparam InputMessageType     incoming Protobuf generated message type
/// @tparam OutputMessageType    outgoing Protobuf generated message type
/// @tparam message_handler      message handler callback
template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
class UartProtobuf : public UartProtobufBase {
public:
    /// Class constructor.
    UartProtobuf(
        const pb_msgdesc_t *input_message_fields,   ///< [in] input message fields description
        const pb_msgdesc_t *output_message_fields,  ///< [in] output message fields description
        uart_t uart_dev,                            ///< [in] UART device
        uint32_t uart_speed=230400U                 ///< [in] UART baud rate
        );

    /// Open serial port.
    bool connect();

    /// Start thread waiting for incoming messages.
    void start_reader();

    void uart_rx_cb(uint8_t data) override;

    /// Decode Protobuf incoming message.
    void decode_message(
        uint32_t message_length,                    ///< [in] message length
        uint8_t *buffer                             ///< [in] encoded message
        );

    void message_reader() override;

    /// Send Protobuf message on serial port.
    bool send_message(
        const OutputMessageType &message            ///< [in] message to send
        );

private:
    const pb_msgdesc_t *input_message_fields_;      ///< input message fields description
    const pb_msgdesc_t *output_message_fields_;     ///< output message fields description
    uart_t uart_dev_;                               ///< UART device
    uint32_t uart_speed_;                           ///< UART baud rate

    kernel_pid_t reader_pid_;                       ///< reader thread PID
    char reader_stack_[UARTPB_READER_STACKSIZE];    ///< reader thread stack

    char rx_mem_[INPUT_MESSAGE_LENGTH_MAX];               ///< memory for UART incoming bytes
    ringbuffer_t rx_buf_;                           ///< ring buffer for UART incoming bytes

    uint8_t msg_length_bytes_number_ = 4;           ///< number of bytes needed to encode message length
    uint8_t msg_length_bytes_received_ = 0;         ///< number of bytes received for current message length
    uint32_t msg_length_ = 0;                       ///< message length
    uint32_t msg_bytes_received_ = 0;               ///< number of bytes received for current message
    uint8_t input_buffer_[INPUT_MESSAGE_LENGTH_MAX];      ///< buffer used to decode incoming messages
    uint8_t output_buffer_[OUTPUT_MESSAGE_LENGTH_MAX];    ///< buffer used to encode outgoing messages
};

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
UartProtobuf(
    const pb_msgdesc_t *input_message_fields,
    const pb_msgdesc_t *output_message_fields,
    uart_t uart_dev, uint32_t uart_speed) :
    input_message_fields_(input_message_fields),
    output_message_fields_(output_message_fields),
    uart_dev_(uart_dev), uart_speed_(uart_speed)
{
    ringbuffer_init(&rx_buf_, rx_mem_, INPUT_MESSAGE_LENGTH_MAX);
}

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
bool UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
connect()
{
    int res = uart_init(uart_dev_, uart_speed_, uart_rx_cb_wrapper, (void *)this);
    return res == UART_OK;
}

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
void UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
start_reader()
{
    reader_pid_ = thread_create(
        reader_stack_, sizeof(reader_stack_), UARTPB_READER_PRIO,
        0, message_reader_wrapper, (void *)this, "reader");
}

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
void UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
uart_rx_cb(uint8_t data)
{
    if (msg_length_bytes_received_ < msg_length_bytes_number_) {
        ((uint8_t *)&msg_length_)[msg_length_bytes_received_++] = data;
        msg_bytes_received_ = 0;
        return;
    }

    if (msg_bytes_received_ < msg_length_) {
        ringbuffer_add_one(&rx_buf_, data);
        msg_bytes_received_++;
    }

    if (msg_bytes_received_ == msg_length_) {
        msg_t msg;
        msg.content.value = msg_length_;
        msg_send(&msg, reader_pid_);
        msg_length_bytes_received_ = 0;
    }
}

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
void UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
decode_message(uint32_t message_length, uint8_t *buffer)
{
    InputMessageType message;
    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
    bool status = pb_decode(&stream, input_message_fields_, &message);

    if (!status)
    {
        printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }

    message_handler(message);
}

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
void UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
message_reader()
{
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);
        uint32_t message_length = (uint32_t)msg.content.value;
        assert(message_length <= INPUT_MESSAGE_LENGTH_MAX);
        ringbuffer_get(&rx_buf_, (char *)input_buffer_, message_length);
        decode_message(message_length, input_buffer_);
    }
}

template <typename InputMessageType, typename OutputMessageType,
          void (*message_handler)(const InputMessageType &)>
bool UartProtobuf<InputMessageType, OutputMessageType, message_handler>::
send_message(const OutputMessageType &message)
{
    size_t length;
    bool status = pb_get_encoded_size(&length, output_message_fields_, &message);
    if (!status)
    {
        printf("Encoding failed: cannot get encoded message size.\n");
        return false;
    }
    assert(length <= OUTPUT_MESSAGE_LENGTH_MAX);

    pb_ostream_t stream = pb_ostream_from_buffer(output_buffer_, length);
    status = pb_encode(&stream, output_message_fields_, &message);

    if (!status)
    {
        printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    uart_write(uart_dev_, (uint8_t *)&stream.bytes_written, 4);
    uart_write(uart_dev_, output_buffer_, stream.bytes_written);

    return true;
}

} // namespace uartpb

} // namespace cogip

/// @}
