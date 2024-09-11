#pragma once

#include "etl/math_constants.h"

// RIOT includes
#include "periph/qdec.h"

// Module includes
#include "EncoderInterface.hpp"
namespace cogip {

namespace encoder {

class Encoder : public EncoderInterface {
public:
    explicit Encoder(uint8_t id, EncoderMode mode, int32_t pulse_per_rev): EncoderInterface(mode, pulse_per_rev), id_(id) {}

    int setup()
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
            printf("Invalid QDEC mode (%u)\n", id_);
            return -1;
        }

        /* Setup qdec peripheral */
        int error = qdec_init(qdec, qdec_mode, NULL, NULL);
        if (error) {
            printf("QDEC %u not initialized, error=%d !!!\n", id_, error);
        }

        return error;
    }

    /**
     * @brief Get the pulses counted by the encoder since the last call.
     *
     * @return int32_t counter value in ticks
     */
    int32_t read_and_reset() override { return qdec_read_and_reset(id_); }

    /**
     * @brief Reset encoder counter
     *
     */
    void reset() override { (void)read_and_reset(); };
    
private:
    uint8_t id_;
};

} // namespace encoder

} // namespace cogip

/// @}
