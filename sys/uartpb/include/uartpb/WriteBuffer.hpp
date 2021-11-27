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

private:
    uint8_t data_[UARTPB_OUTPUT_MESSAGE_LENGTH_MAX];  ///< array in which the serialized data is stored
    uint32_t write_index_;                            ///< number of bytes currently serialized in the array
};

} // namespace uartpb

} // namespace cogip
