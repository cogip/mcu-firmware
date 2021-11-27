// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     sys_uartpb
/// @brief       Read buffer for EmbeddedProto.
/// @{
/// @file
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include <cstdint>
#include "uartpb.hpp"
#include "ReadBufferInterface.h"

namespace cogip {

namespace uartpb {

/// ReadBuffer class used to decode Protobuf messages
class ReadBuffer : public EmbeddedProto::ReadBufferInterface
{
public:
    /// Class constructor
    ReadBuffer();

    ~ReadBuffer() override = default;

    uint32_t get_size() const override;

    uint32_t get_max_size() const override;

    bool peek(uint8_t &byte) const override;

    void advance() override;

    void advance(const uint32_t n) override;

    bool pop(uint8_t &byte) override;

    /// Return a pointer to the data array
    uint8_t * get_data_array();

    /// Return a non constant reference to the number of bytes written to the data array.
    uint32_t & get_bytes_written();

    /// Clear all indices, in effect allowing the data to be overwritten.
    void clear();

    /// Push new data into the buffer.
    bool push(uint8_t &byte);

private:
    uint8_t data_[UARTPB_INPUT_MESSAGE_LENGTH_MAX];  ///< array in which the data received over uart is stored
    uint32_t write_index_;                           ///< number of bytes currently received and stored in the data array
    uint32_t read_index_;                            ///< number of bytes read from the data array
};

} // namespace uartpb

} // namespace cogip
