// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_canpb
/// @brief       Exchange Protobuf messages over CAN module.
/// @details     This module provides a class that controls an CAN to send and
///              receive Protobuf messages using EmbbededProto.
///              Each library/application using canpb must define a unique id
///              (32-bit integer of type cogip::canpb::uuid_t) and registrer a
///              callback function to send or receive their own messages. See
///              `examples/canb`.
/// @{
/// @file
/// @author      Gilles DOFFE <g.doffe@gmail.com>
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "etl/delegate.h"

// RIOT includes
#include "can/conn/raw.h"
#include "etl/map.h"
#include "periph/can.h"
#include "ringbuffer.h"
#include "thread.h"
#include <mutex.h>

#include "canpb/ReadBuffer.hpp"
#include "canpb/WriteBuffer.hpp"
#include "canpb/canpb.hpp"

#include "MessageInterface.h"

// Double the size of the ringbuffer receiving input bytes,
// so we can continue receiving data even if the thread decoding
// the message has not consumed previous message immediately.
#define CAN_BUFFER_SIZE CANPB_INPUT_MESSAGE_LENGTH_MAX * 2

namespace cogip {

namespace canpb {

/// Custom type for uuids
using uuid_t = uint32_t;

/// Prototype for incoming Protobuf message handlers
using message_handler_t = etl::delegate<void(cogip::canpb::ReadBuffer&)>;

/// Thread function decoding incoming Protobuf messages.
/// This wrapper is used to call the message_reader() function from CanProtobuf
/// class from C context, passing the CanProtobuf instance pointer as first
/// parameter.
void* message_reader_wrapper(void* arg ///< [in] pointer to CanProtobuf instance
);

/// Generic CAN Protobuf communication class.
class CanProtobuf
{
  public:
    /// Class constructor.
    explicit CanProtobuf(uint8_t can_interface_number ///< [in] CAN device
    );

    /// Initialize CAN connection.
    /// @return true if CAN connection is initialized, false otherwise
    bool init(struct can_filter* filter);

    /// Start thread waiting for incoming messages.
    void start_reader();

    /// Function call for each incomming frame on CAN port.
    void can_rx_cb(uint8_t data ///< [in] incoming data
    );

    /// Wait and decode incoming Protobuf messages.
    void message_reader();

    /// Send CAN message.
    /// @return true if message was encoded and sent, false otherwise
    bool send_message(uuid_t uuid, ///< [in] message uuid
                      const EmbeddedProto::MessageInterface* message = nullptr
                      ///< [in] message to send
    );

    /// Associate a message handle to a specific uuid
    void register_message_handler(uuid_t uuid,              ///< [in] message uuid
                                  message_handler_t handler ///< [in] message handler
    );

  private:
    conn_can_raw_t conn_can_raw_;  ///< Raw CAN connection
    uint8_t can_interface_number_; ///< CAN interface number
    kernel_pid_t reader_pid_;      ///< reader thread PID
    ringbuffer_t rx_buf_;          ///< ring buffer for CAN incoming bytes
    uint32_t msg_length_ = 0;      ///< message length
    mutex_t mutex_ = MUTEX_INIT;   ///< mutex protecting CAN port access
    etl::map<uuid_t, message_handler_t, CANPB_MAX_HANDLERS> message_handlers_;
    ///< callbacks to process the message after decoding
    char reader_stack_[CANPB_READER_STACKSIZE]; ///< reader thread stack
    char rx_mem_[CAN_BUFFER_SIZE];              ///< memory for CAN incoming bytes
    ReadBuffer read_buffer_;                    ///< buffer used to decode a message
    WriteBuffer write_buffer_;                  ///< buffer used to encode a message
};

} // namespace canpb

} // namespace cogip

/// @}
