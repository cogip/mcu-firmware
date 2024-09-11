#pragma once

#include "etl/math_constants.h"

// RIOT includes
#include "EncoderInterface.hpp"

// Module includes
#include "EncoderParams.hpp"
#include "periph/qdec.h"

namespace cogip {

namespace encoder {

class Encoder : public EncoderInterface {
public:
    explicit Encoder(EncoderParams &parameters) : _parameters(parameters) {}

    int setup()
    {
        qdec_t qdec;
        qdec_mode_t mode;

        switch (_parameters.id()) {
        case EncoderID::LEFT_ENCODER_ID:
            qdec = QDEC_DEV(0);
            break;
        case EncoderID::RIGHT_ENCODER_ID:
            qdec = QDEC_DEV(1);
            break;
        default:
            printf("Invalid QDEC id (%u)\n", (uint8_t)_parameters.id());
            return -1;
        }

        switch (_parameters.mode()) {
        case EncoderMode::ENCODER_MODE_X1:
            mode = QDEC_X1;
            break;
        case EncoderMode::ENCODER_MODE_X2:
            mode = QDEC_X2;
            break;
        case EncoderMode::ENCODER_MODE_X4:
            mode = QDEC_X4;
            break;
        default:
            printf("Invalid QDEC mode (%u)\n", (uint8_t)_parameters.id());
            return -1;
        }

        /* Setup qdec peripheral */
        int error = qdec_init(qdec, mode, NULL, NULL);
        if (error) {
            printf("QDEC %u not initialized, error=%d !!!\n", (uint8_t)_parameters.id(), error);
        }

        return error;
    }

    /**
     * @brief Get the distance measured by the encoder since the last call.
     *
     * @note This can become a speed in mm/Tech once periodically called.
     *
     * @return float distance in mm
     */
    double get_traveled_distance() override
    {
        int32_t counter = qdec_read_and_reset((qdec_t)_parameters.id());

        return (double)counter / _parameters.pulse_per_rev() *
               (etl::math::pi * _parameters.wheel_diameter());
    }

    /**
     * @brief Get the encoder raw counter value
     *
     * @return double pulse since last call
     */
    int32_t get_raw_counter() { return qdec_read_and_reset((qdec_t)_parameters.id()); }

    /**
     * @brief Reset encoder counter
     *
     */
    void reset() override { qdec_read_and_reset((qdec_t)_parameters.id()); };

private:
    EncoderParams &_parameters;
};

} // namespace encoder

} // namespace cogip

/// @}
