#pragma once

#include "etl/math_constants.h"

// RIOT includes
#include "periph/qdec.h"

// Module includes
#include "EncoderInterface.hpp"
namespace cogip {

namespace encoder {

class EncoderQDEC : public EncoderInterface
{
  public:
    ///
    /// @brief Construct a new Encoder object
    /// @note Encoder pulse per revolution must be set according to chosen mode
    /// eg. For a 2500 PPR encoder:
    ///              - ENCODER_MODE_X1 -> pulse_per_rev = 2500
    ///              - ENCODER_MODE_X2 -> pulse_per_rev = 5000
    ///              - ENCODER_MODE_X4 -> pulse_per_rev = 10000
    /// @param mode
    /// @param pulse_per_rev
    ///
    explicit EncoderQDEC(uint8_t id, EncoderMode mode, int32_t pulse_per_rev)
        : EncoderInterface(mode, pulse_per_rev), id_(id)
    {
    }

    ///
    /// @brief Setup encoder
    ///
    /// @return int negative on error. 0 on succes
    ///
    int init() override;

    ///
    /// @brief Get the pulses counted by the encoder since the last call.
    ///
    /// @return int32_t counter value in ticks
    ///
    int32_t read_and_reset() override;

    ///
    /// @brief Reset encoder counter
    ///
    ///
    void reset() override;

  private:
    uint8_t id_;
};

} // namespace encoder

} // namespace cogip

/// @}
