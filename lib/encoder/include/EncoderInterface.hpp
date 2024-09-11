#pragma once

namespace cogip {

namespace encoder {

class EncoderInterface {
public:
    /**
     * @brief Get the distance measured by the encoder since the last call.
     *
     * @note This can become a speed in mm/Tech once periodically called.
     *
     * @return float distance in mm
     */
    virtual double get_traveled_distance() = 0;

    /**
     * @brief Reset encoder counter
     *
     */
    virtual void reset() = 0;
};

} // namespace encoder

} // namespace cogip

/// @}
