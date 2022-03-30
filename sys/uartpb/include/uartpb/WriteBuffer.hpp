// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_uartpb
/// @brief       Write buffer for EmbeddedProto.
/// @{
/// @file
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include <cstdint>
#include "uartpb.hpp"
#include "WriteBufferInterface.h"

/// Reserved space at the begining of the serialization buffer to store meta data.
///   - 1 byte for message type
#define UARTPB_METADATA_SIZE 1

/// Size of the Protobuf serialization buffer
#define UARTPB_SERIALIZATION_BUFFER_SIZE (UARTPB_OUTPUT_MESSAGE_LENGTH_MAX + UARTPB_METADATA_SIZE)

/// Size of the base64 encoding buffer
#define UARTPB_BASE64_BUFFER_SIZE (UARTPB_SERIALIZATION_BUFFER_SIZE * 2)

namespace cogip {

namespace uartpb {

/// WriteBuffer class used to encode Protobuf messages
class WriteBuffer : public EmbeddedProto::WriteBufferInterface
{
public:
    /// Class constructor
    WriteBuffer() = default;

    ~WriteBuffer() override = default;

    virtual void clear() override;

    virtual uint32_t get_size() const override;

    virtual uint32_t get_max_size() const override;

    virtual uint32_t get_available_size() const override;

    virtual bool push(const uint8_t byte) override;

    virtual bool push(const uint8_t *bytes, const uint32_t length) override;

    /// Return a pointer to the data array.
    uint8_t * get_data();

    /// Set type of the protobuf message
    void set_message_type(uint8_t type);

    /// Encode the data buffer in base64 before transmission over UART.
    /// @return size of encoded message, 0 in case of failure.
    size_t base64_encode();

    /// Return a pointer to the data array.
    uint8_t * get_base64_data();

private:
    ///< array in which the serialized data is stored
    uint8_t data_[UARTPB_SERIALIZATION_BUFFER_SIZE];
    ///< array in which the base64 encoded serialized data is stored
    uint8_t base64_data_[UARTPB_BASE64_BUFFER_SIZE];
    ///< number of bytes currently serialized in the array
    uint32_t write_index_;
};

} // namespace uartpb

} // namespace cogip
