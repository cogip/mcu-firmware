// Copyright (C) 2025 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @brief       QDEC encoder implementation
/// @author      COGIP Robotics

// RIOT includes
#include "periph/qdec.h"

#include "encoder/EncoderQDEC.hpp"

namespace cogip {

namespace encoder {

int EncoderQDEC::init()
{
    qdec_t qdec;
    qdec_mode_t qdec_mode;

    qdec = QDEC_DEV(id_);

    switch (mode_) {
    case EncoderMode::ENCODER_MODE_X1:
        qdec_mode = QDEC_X1;
        break;
    case EncoderMode::ENCODER_MODE_X2:
        qdec_mode = QDEC_X2;
        break;
    case EncoderMode::ENCODER_MODE_X4:
        qdec_mode = QDEC_X4;
        break;
    default:
        return -1;
    }

    /* Setup QDEC peripheral */
    return qdec_init(qdec, qdec_mode, NULL, NULL);
}

int32_t EncoderQDEC::read_and_reset()
{
    int32_t delta = qdec_read_and_reset(id_);
    // Overflow is not a concern: int64_t range (~9.2 quintillion) far exceeds realistic encoder
    // counts
    counter_ += delta;
    return delta;
}

void EncoderQDEC::reset()
{
    static_cast<void>(read_and_reset());
    counter_ = 0;
}

} // namespace encoder

} // namespace cogip
