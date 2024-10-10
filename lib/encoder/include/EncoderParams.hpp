#pragma once

namespace cogip {

namespace encoder {

enum class EncoderID : uint8_t {
    LEFT_ENCODER_ID  = 0,
    RIGHT_ENCODER_ID = 1,
};

enum class EncoderMode : uint8_t {
    /*
     * With X1 encoding, either the rising (aka leading) or the falling (aka following) edge of
     * channel A is counted. If channel A leads channel B, the rising edge is counted, and the
     * movement is forward, or clockwise. Conversely, if channel B leads channel A, the falling edge
     * is counted, and the movement is backwards, or counterclockwise.
     */
    ENCODER_MODE_X1 = 0,
    /*
     * When X2 encoding is used, both the rising and falling edges of channel A are counted. This
     * doubles the number of pulses that are counted for each rotation or linear distance, which in
     * turn doubles the encoder’s resolution.
     */
    ENCODER_MODE_X2 = 1,
    /*
     * X4 encoding goes one step further, to count both the rising and falling edges of both
     * channels A and B, which quadruples the number of pulses and increases resolution by four
     * times.
     */
    ENCODER_MODE_X4 = 2,
};

class EncoderParams {
public:
    /**
     * @brief Construct a new Encoder Params object
     *
     * @note  Encoder pulse per revolution must be set according to chosen mode
     *        eg. For a 2500 PPR encoder:
     *               - ENCODER_MODE_X1 -> pulse_per_rev = 2500
     *               - ENCODER_MODE_X2 -> pulse_per_rev = 5000
     *               - ENCODER_MODE_X4 -> pulse_per_rev = 10000
     *
     * @param id Encoder ID
     * @param mode Encoder mode
     * @param wheel_diameter Wheel diameter in millimeters
     * @param pulse_per_rev Encoder total pulses per revolution depending on mode choosen
     */
    EncoderParams(EncoderID id, EncoderMode mode, double wheel_diameter, double pulse_per_rev)
        : _id(id), _mode(mode), _wheel_diameter(wheel_diameter), _pulse_per_rev(pulse_per_rev) {};

    /**
     * @brief Return encoder ID
     *
     * @note possible values:
     *          - LEFT_ENCODER_ID
     *          - RIGHT_ENCODER_ID
     *
     * @return EncoderID
     */
    EncoderID id() const { return _id; }

    /**
     * @brief Return encoder Mode
     *
     * @note possible values:
     *          - ENCODER_MODE_X1
     *          - ENCODER_MODE_X2
     *          - ENCODER_MODE_X4
     *
     * @return EncoderMode
     */
    EncoderMode mode() const { return _mode; }

    /**
     * @brief Return encoder wheel diameter value in mm.
     *
     * @return double
     */
    double wheel_diameter() const { return _wheel_diameter; }

    /**
     * @brief Return encoder pulse per revolution
     *
     * @return double
     */
    double pulse_per_rev() const { return _pulse_per_rev; }

private:
    EncoderID _id;
    EncoderMode _mode;
    double _wheel_diameter;
    double _pulse_per_rev;
};

} // namespace encoder

} // namespace cogip

/// @}
