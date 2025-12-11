#pragma once

#include <cstdint>

#include "etl/math_constants.h"

namespace cogip {

namespace encoder {

enum class EncoderMode : uint8_t {
    /// With X1 encoding, either the rising (aka leading) or the falling (aka
    /// following) edge of
    /// channel A is counted. If channel A leads channel B, the rising edge is
    /// counted, and the
    /// movement is forward, or clockwise. Conversely, if channel B leads channel
    /// A, the falling edge
    /// is counted, and the movement is backwards, or counterclockwise.
    ENCODER_MODE_X1 = 0,

    /// When X2 encoding is used, both the rising and falling edges of channel A
    /// are counted. This
    /// doubles the number of pulses that are counted for each rotation or linear
    /// distance, which in
    /// turn doubles the encoder’s resolution.
    ENCODER_MODE_X2 = 1,

    /// X4 encoding goes one step further, to count both the rising and falling
    /// edges of both
    /// channels A and B, which quadruples the number of pulses and increases
    /// resolution by four
    /// times.
    ENCODER_MODE_X4 = 2,
};

class EncoderInterface
{
  public:
    ///
    /// @brief Construct a new Encoder Interface object
    /// @note Encoder pulse per revolution must be set according to chosen mode
    /// eg. For a 2500 PPR encoder:
    ///              - ENCODER_MODE_X1 -> pulse_per_rev = 2500
    ///              - ENCODER_MODE_X2 -> pulse_per_rev = 5000
    ///              - ENCODER_MODE_X4 -> pulse_per_rev = 10000
    /// @param mode
    /// @param pulse_per_rev
    ///
    EncoderInterface(EncoderMode mode, int32_t pulse_per_rev)
        : mode_(mode), pulse_per_rev_(pulse_per_rev), counter_(0)
    {
    }

    ///
    /// @brief Init low level encoder driver.
    ///
    /// @return 0 on success, negative value on failure.
    ///
    virtual int init() = 0;

    ///
    /// @brief Get the pulses counted by the encoder since the last call.
    ///
    /// @return int32_t int32_t counter value in ticks
    ///
    virtual int32_t read_and_reset() = 0;

    ///
    /// @brief Reset encoder counter
    ///
    virtual void reset() = 0;

    ///
    /// @brief Get the accumulated encoder counter value.
    ///
    /// @return int64_t accumulated counter value in ticks.
    ///
    int64_t counter()
    {
        return counter_;
    }

    ///
    /// @brief Get the angle measured by the encoder since the last call.
    ///
    /// @return float traveled angle since last call (rad).
    ///
    float get_angle_and_reset()
    {
        return ((float)read_and_reset() / (float)pulse_per_rev_) * etl::math::pi;
    }

  protected:
    const EncoderMode mode_;
    const int32_t pulse_per_rev_;
    int64_t counter_;
};

} // namespace encoder

} // namespace cogip

/// @}
