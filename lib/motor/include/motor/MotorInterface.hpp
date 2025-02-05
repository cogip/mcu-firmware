#pragma once

namespace cogip {

namespace motor {

class MotorInterface {
public:
    /// @brief Virtual destructor
    ~MotorInterface()
    {
    }

    /// @brief Enable the motor
    /// @return 0 on success, negative on error
    virtual int enable() = 0;

    /// @brief Enable the motor
    /// @return 0 on success, negative on error
    virtual int disable() = 0;

    /// @brief Set motor speed
    /// @param speed speed in % [-100; 100]
    /// @return  0 on success, negative on error
    virtual int set_speed(double speed) = 0;

    /// @brief break the motor
    /// @return 0 on success, negative on error
    virtual int brake() = 0;
};

} // namespace motor

} // namespace cogip

// @}
